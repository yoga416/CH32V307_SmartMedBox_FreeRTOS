#include "bsp_max_30102_port.h"
#include "ch32v30x.h"
#include "task.h"
#include "queue.h"
#include <string.h>
#include "bsp_max_30102_handler.h"
#include "debug.h"
#include "Middle_ring_buffer.h"
#include "app_task.h"
#include "bsp_tianwen_reg.h"
bsp_max30102_driver_t g_max30102;
SemaphoreHandle_t xSem_MAX30102_Exti = NULL;
// 定义一个静态重试计数器
static uint8_t s_hr_retry_count = 0; 
// 定义最大重试次数（共 1 次）
#define MAX_HR_RETRIES 0

// 定义滑动滤波窗口大小（建议 3 到 5，数值越大越平滑但响应越慢）
#define HR_SPO2_FILTER_SIZE 3

// 静态全局历史数组，用于滤波
static int32_t g_hr_history[HR_SPO2_FILTER_SIZE] = {0};
static float   g_spo2_history[HR_SPO2_FILTER_SIZE] = {0.0f};
static uint8_t g_filter_idx = 0;

// 在文件上方定义超时阈值
#define I2C_TIMEOUT_MAX 50000

/* ---------------- I2C 总线恢复函数 ---------------- */
static uint8_t s_i2c_recovery_count = 0;
static void i2c_bus_recovery(void)
{
    s_i2c_recovery_count++;
    printf("[I2C] Bus recovery attempt #%d...\r\n", s_i2c_recovery_count);

    // 1. 禁用 I2C 外设
    I2C_Cmd(I2C1, DISABLE);

    // 2. 软件复位 I2C
    I2C_SoftwareResetCmd(I2C1, ENABLE);
    for (volatile int k = 0; k < 2000; k++);
    I2C_SoftwareResetCmd(I2C1, DISABLE);

    // 3. 切为推挽输出，发时钟脉冲解锁从机
    GPIO_InitTypeDef g = {
        .GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9,
        .GPIO_Mode = GPIO_Mode_Out_PP,
        .GPIO_Speed = GPIO_Speed_50MHz
    };
    GPIO_Init(GPIOB, &g);

    // 先拉高
    GPIO_SetBits(GPIOB, GPIO_Pin_8 | GPIO_Pin_9);
    Delay_Us(10);

    // 发 9 个 SCL 脉冲
    for (int i = 0; i < 9; i++) {
        GPIO_ResetBits(GPIOB, GPIO_Pin_8);
        Delay_Us(5);
        GPIO_SetBits(GPIOB, GPIO_Pin_8);
        Delay_Us(5);
    }

    // STOP 信号
    GPIO_ResetBits(GPIOB, GPIO_Pin_9);
    Delay_Us(5);
    GPIO_SetBits(GPIOB, GPIO_Pin_8);
    Delay_Us(5);
    GPIO_SetBits(GPIOB, GPIO_Pin_9);
    Delay_Us(5);

    // 4. 切回复用开漏
    GPIO_PinRemapConfig(GPIO_Remap_I2C1, ENABLE);
    g.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(GPIOB, &g);

    // 5. 重新初始化 I2C
    I2C_InitTypeDef i = {
        .I2C_ClockSpeed = 400000,
        .I2C_Mode = I2C_Mode_I2C,
        .I2C_DutyCycle = I2C_DutyCycle_2,
        .I2C_OwnAddress1 = 0x00,
        .I2C_Ack = I2C_Ack_Enable,
        .I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit
    };
    I2C_Init(I2C1, &i);
    I2C_Cmd(I2C1, ENABLE);

    Delay_Ms(20);
    printf("[I2C] Bus recovery done.\r\n");
}

/* ---------------- 硬件 I2C 阻塞写 (带超时和打印打印) ---------------- */
static MAX_DRIVER_Status_t hw_i2c_write(void *context, uint16_t dev_addr, uint8_t reg, uint8_t data) {
    uint32_t timeout;

    timeout = I2C_TIMEOUT_MAX;
    while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY)) {
        if(--timeout == 0) {
            printf("[I2C_ERR] Write: BUSY timeout\r\n");
            i2c_bus_recovery();
            return MAX_DRIVER_ERROR;
        }
    }

    I2C_GenerateSTART(I2C1, ENABLE);
    timeout = I2C_TIMEOUT_MAX;
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) {
        if(--timeout == 0) { printf("[I2C_ERR] Write: START timeout\r\n"); return MAX_DRIVER_ERROR; }
    }

    I2C_Send7bitAddress(I2C1, dev_addr, I2C_Direction_Transmitter);
    timeout = I2C_TIMEOUT_MAX;
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
        if(I2C_GetFlagStatus(I2C1, I2C_FLAG_AF)) {
            printf("[I2C_ERR] Write: No ACK from device (addr=0x%02X)\r\n", dev_addr);
            I2C_ClearFlag(I2C1, I2C_FLAG_AF);
            I2C_GenerateSTOP(I2C1, ENABLE);
            return MAX_DRIVER_ERROR;
        }
        if(--timeout == 0) { printf("[I2C_ERR] Write: ADDR(TX) timeout\r\n"); return MAX_DRIVER_ERROR; }
    }

    I2C_SendData(I2C1, reg);
    timeout = I2C_TIMEOUT_MAX;
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
        if(--timeout == 0) { printf("[I2C_ERR] Write: REG_TX timeout\r\n"); return MAX_DRIVER_ERROR; }
    }

    I2C_SendData(I2C1, data);
    timeout = I2C_TIMEOUT_MAX;
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
        if(--timeout == 0) { printf("[I2C_ERR] Write: DATA_TX timeout\r\n"); return MAX_DRIVER_ERROR; }
    }

    I2C_GenerateSTOP(I2C1, ENABLE);
    return MAX_DRIVER_OK;
}

static MAX_DRIVER_Status_t hw_i2c_read(void *context, uint16_t dev_addr, uint8_t reg, uint8_t *p_data) {
    uint32_t timeout;

    timeout = I2C_TIMEOUT_MAX;
    while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY)) {
        if(--timeout == 0) {
            printf("[I2C_ERR] Read: BUSY timeout\r\n");
            i2c_bus_recovery();
            return MAX_DRIVER_ERROR;
        }
    }

    I2C_GenerateSTART(I2C1, ENABLE);
    timeout = I2C_TIMEOUT_MAX;
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) {
        if(--timeout == 0) { printf("[I2C_ERR] Read: START 1 timeout\r\n"); return MAX_DRIVER_ERROR; }
    }

    I2C_Send7bitAddress(I2C1, dev_addr, I2C_Direction_Transmitter);
    timeout = I2C_TIMEOUT_MAX;
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
        if(I2C_GetFlagStatus(I2C1, I2C_FLAG_AF)) {
            printf("[I2C_ERR] Read: No ACK from device (addr=0x%02X)\r\n", dev_addr);
            I2C_ClearFlag(I2C1, I2C_FLAG_AF);
            I2C_GenerateSTOP(I2C1, ENABLE);
            return MAX_DRIVER_ERROR;
        }
        if(--timeout == 0) { printf("[I2C_ERR] Read: ADDR(TX) timeout\r\n"); return MAX_DRIVER_ERROR; }
    }

    I2C_SendData(I2C1, reg);
    timeout = I2C_TIMEOUT_MAX;
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
        if(--timeout == 0) { printf("[I2C_ERR] Read: REG_TX timeout\r\n"); return MAX_DRIVER_ERROR; }
    }

    I2C_GenerateSTART(I2C1, ENABLE);
    timeout = I2C_TIMEOUT_MAX;
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) {
        if(--timeout == 0) { printf("[I2C_ERR] Read: START 2 timeout\r\n"); return MAX_DRIVER_ERROR; }
    }

    I2C_Send7bitAddress(I2C1, dev_addr, I2C_Direction_Receiver);
    timeout = I2C_TIMEOUT_MAX;
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) {
        if(I2C_GetFlagStatus(I2C1, I2C_FLAG_AF)) {
            printf("[I2C_ERR] Read: No ACK from device (addr=0x%02X)\r\n", dev_addr);
            I2C_ClearFlag(I2C1, I2C_FLAG_AF);
            I2C_GenerateSTOP(I2C1, ENABLE);
            return MAX_DRIVER_ERROR;
        }
        if(--timeout == 0) { printf("[I2C_ERR] Read: ADDR(RX) timeout\r\n"); return MAX_DRIVER_ERROR; }
    }
    
    I2C_AcknowledgeConfig(I2C1, DISABLE); // 单字节读完直接 NACK
    timeout = I2C_TIMEOUT_MAX;
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)) {
        if(--timeout == 0) { printf("[I2C_ERR] Read: DATA_RX timeout\r\n"); return MAX_DRIVER_ERROR; }
    }
    *p_data = I2C_ReceiveData(I2C1);
    
    I2C_GenerateSTOP(I2C1, ENABLE);
    I2C_AcknowledgeConfig(I2C1, ENABLE);
    return MAX_DRIVER_OK;
}


static MAX_DRIVER_Status_t hw_i2c_read_mem(void *context, uint16_t dev_addr, uint8_t reg, uint8_t *p_data, uint16_t size) {
    uint32_t timeout;

    if(size == 0) return MAX_DRIVER_PARAM;

    timeout = I2C_TIMEOUT_MAX;
    while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY)) {
        if(--timeout == 0) {
            printf("[I2C_ERR] ReadMem: BUSY timeout\r\n");
            i2c_bus_recovery();
            return MAX_DRIVER_ERROR;
        }
    }

    I2C_GenerateSTART(I2C1, ENABLE);
    timeout = I2C_TIMEOUT_MAX;
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) {
        if(--timeout == 0) { printf("[I2C_ERR] ReadMem: START 1 timeout\r\n"); return MAX_DRIVER_ERROR; }
    }

    I2C_Send7bitAddress(I2C1, dev_addr, I2C_Direction_Transmitter);
    timeout = I2C_TIMEOUT_MAX;
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
        if(I2C_GetFlagStatus(I2C1, I2C_FLAG_AF)) {
            printf("[I2C_ERR] ReadMem: No ACK from device (addr=0x%02X)\r\n", dev_addr);
            I2C_ClearFlag(I2C1, I2C_FLAG_AF);
            I2C_GenerateSTOP(I2C1, ENABLE);
            return MAX_DRIVER_ERROR;
        }
        if(--timeout == 0) { printf("[I2C_ERR] ReadMem: ADDR(TX) timeout\r\n"); return MAX_DRIVER_ERROR; }
    }

    I2C_SendData(I2C1, reg);
    timeout = I2C_TIMEOUT_MAX;
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
        if(--timeout == 0) { printf("[I2C_ERR] ReadMem: REG_TX timeout\r\n"); return MAX_DRIVER_ERROR; }
    }

    I2C_GenerateSTART(I2C1, ENABLE);
    timeout = I2C_TIMEOUT_MAX;
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) {
        if(--timeout == 0) { printf("[I2C_ERR] ReadMem: START 2 timeout\r\n"); return MAX_DRIVER_ERROR; }
    }

    I2C_Send7bitAddress(I2C1, dev_addr, I2C_Direction_Receiver);
    timeout = I2C_TIMEOUT_MAX;
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) {
        if(I2C_GetFlagStatus(I2C1, I2C_FLAG_AF)) {
            printf("[I2C_ERR] ReadMem: No ACK from device (addr=0x%02X)\r\n", dev_addr);
            I2C_ClearFlag(I2C1, I2C_FLAG_AF);
            I2C_GenerateSTOP(I2C1, ENABLE);
            return MAX_DRIVER_ERROR;
        }
        if(--timeout == 0) { printf("[I2C_ERR] ReadMem: ADDR(RX) timeout\r\n"); return MAX_DRIVER_ERROR; }
    }
    
    for(uint16_t i = 0; i < size; i++) {
        if(i == size - 1) {
            // 接收最后一个字节前，发送 NACK
            I2C_AcknowledgeConfig(I2C1, DISABLE);
        }
        timeout = I2C_TIMEOUT_MAX;
        while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)) {
            if(--timeout == 0) { printf("[I2C_ERR] ReadMem: DATA_RX timeout (byte %d)\r\n", i); return MAX_DRIVER_ERROR; }
        }
        p_data[i] = I2C_ReceiveData(I2C1);
    }
    
    I2C_GenerateSTOP(I2C1, ENABLE);
    I2C_AcknowledgeConfig(I2C1, ENABLE); // 恢复应答机制
    return MAX_DRIVER_OK;
}


// 实例化一个全局的 Handler 对象
bsp_max_handler_t g_max30102_handler;

static MAX30102_HANDLER_Status_t os_queue_create(uint32_t const item_num, uint32_t const item_size, void **const queue_handler) {
    *queue_handler = (void *)xQueueCreate(item_num, item_size);
    return (*queue_handler != NULL) ? MAX_HANDLER_OK : MAX_HANDLER_ERROR;
}

static MAX30102_HANDLER_Status_t os_queue_put(void * const queue_handler, void const *const item, uint32_t timeout_ms) {
    TickType_t ticks = (timeout_ms == portMAX_DELAY) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    BaseType_t res = xQueueSend((QueueHandle_t)queue_handler, item, ticks);
    return (res == pdPASS) ? MAX_HANDLER_OK : MAX_HANDLER_TIMEOUT;
}

static MAX30102_HANDLER_Status_t os_queue_get(void * const queue_handler, void * const item, uint32_t timeout_ms) {
    TickType_t ticks = (timeout_ms == portMAX_DELAY) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    BaseType_t res = xQueueReceive((QueueHandle_t)queue_handler, item, ticks);
    return (res == pdPASS) ? MAX_HANDLER_OK : MAX_HANDLER_TIMEOUT;
}

static void os_delay(uint32_t const ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}
/* ---------------- 端口与驱动初始化 ---------------- */
void bsp_max30102_port_init(void) {
    // 1. 开启时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

    // 2. 先把引脚设为普通推挽输出（强力驱动），确保能拉高总线
    GPIO_InitTypeDef g = {
        .GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9,
        .GPIO_Mode = GPIO_Mode_Out_PP, // 先用推挽，强行解锁
        .GPIO_Speed = GPIO_Speed_50MHz
    };
    GPIO_Init(GPIOB, &g);

    // 补充：确保 SDA(PB9) 在发时钟脉冲时处于高电平状态，否则从机无法释放总线！
    GPIO_SetBits(GPIOB, GPIO_Pin_9); 
    Delay_Us(10);

    // 3. 执行 9 个脉冲解锁逻辑
    for (int i = 0; i < 9; i++) {
        GPIO_ResetBits(GPIOB, GPIO_Pin_8); 
        Delay_Us(5);
        GPIO_SetBits(GPIOB, GPIO_Pin_8);   // 确保最后一次是高
        Delay_Us(5);
    }

    // 4. 产生一个标准的 STOP 信号，确保从机状态机归位
    GPIO_ResetBits(GPIOB, GPIO_Pin_9); // SDA 低
    Delay_Us(5);
    GPIO_SetBits(GPIOB, GPIO_Pin_8);   // SCL 高
    Delay_Us(5);
    GPIO_SetBits(GPIOB, GPIO_Pin_9);   // SDA 高（STOP 信号完成）
    Delay_Us(5);

    // 5. 【关键：重要修复】在配置硬件 I2C 之前，先彻底复位 I2C 外设
    I2C_SoftwareResetCmd(I2C1, ENABLE);
    // 稍微延时一下，给硬件复位的时间
    for(volatile int k=0; k<1000; k++); 
    I2C_SoftwareResetCmd(I2C1, DISABLE);

    // 6. 切换为硬件复用开漏模式
    GPIO_PinRemapConfig(GPIO_Remap_I2C1, ENABLE); 
    g.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(GPIOB, &g);

    // 7. 正式初始化硬件 I2C 寄存器
    I2C_InitTypeDef i = {
        .I2C_ClockSpeed = 400000, // 建议先用 100k 调试
        .I2C_Mode = I2C_Mode_I2C,
        .I2C_DutyCycle = I2C_DutyCycle_2,
        .I2C_OwnAddress1 = 0x00,
        .I2C_Ack = I2C_Ack_Enable,
        .I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit
    };
    I2C_Init(I2C1, &i);
    I2C_Cmd(I2C1, ENABLE);
  
    // 6. EXTI PC6 初始化 (保持你原有的不变)
    g.GPIO_Pin = GPIO_Pin_6; 
    g.GPIO_Mode = GPIO_Mode_IPU; 
    GPIO_Init(GPIOC, &g);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource6);
    EXTI_InitTypeDef e = { 
        .EXTI_Line = EXTI_Line6, 
        .EXTI_Mode = EXTI_Mode_Interrupt, 
        .EXTI_Trigger = EXTI_Trigger_Falling, 
        .EXTI_LineCmd = ENABLE 
    };
    EXTI_Init(&e);

    // 设置中断优先级并使能 (必须低于等于 FreeRTOS 的 configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY)
    NVIC_InitTypeDef nvic_init;
    nvic_init.NVIC_IRQChannel = EXTI9_5_IRQn;
    nvic_init.NVIC_IRQChannelPreemptionPriority = 5; // 优先级需 >= 5 才能安全调 FreeRTOS API
    nvic_init.NVIC_IRQChannelSubPriority = 0;
    nvic_init.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_init);

    xSem_MAX30102_Exti = xSemaphoreCreateBinary();

    static max_hw_interface_t hw_if = {
        .hi2c = I2C1,
        .pf_write_reg = hw_i2c_write,
        .pf_read_reg  = hw_i2c_read,
        .pf_read_mem  = hw_i2c_read_mem
    };
    static max_sys_interface_t sys_if = { .pf_delay_ms = Delay_Ms };

    if (bsp_max30102_inst(&g_max30102, &hw_if, &sys_if) != MAX_DRIVER_OK) {
        printf("[MAX30102] *** WARNING: Sensor init FAILED! Check wiring (VCC/GND/SDA-PB9/SCL-PB8). ***\r\n");
    } else {
        printf("[MAX30102] Sensor init OK (ID=0x15 confirmed).\r\n");
    }
}
// 心率血氧解算结果回调函数
void on_hr_spo2_calculated(int32_t hr, int8_t hr_valid, float spo2, int8_t spo2_valid) 
{
    // 阈值常量定义
    const uint32_t hr_max_threshold = 180;
    const uint32_t hr_min_threshold = 40;
    const float spo2_min_threshold = 70.0f; // 提高血氧阈值更符合生理逻辑
    const float spo2_max_threshold = 100.0f;   
    
    // 1. 总体有效性判断 (将是否为0，以及是否在范围内的判断合并)
    bool is_data_valid = (hr_valid != 0 && spo2_valid != 0 && 
                          hr >= hr_min_threshold && hr <= hr_max_threshold && 
                          spo2 >= spo2_min_threshold && spo2 <= spo2_max_threshold);

    if (is_data_valid) 
    {
        s_hr_retry_count = 0; 

        // 屏蔽掉滑动滤波的过程，直接打印算法输出的原始（未经二次平滑）结果
        APP_LOG("RAW Heart Rate: %ld bpm\r\n", hr);
        
        uint32_t spo2_raw_int = (uint32_t)(spo2 * 10.0f + 0.5f); 
        APP_LOG("RAW SpO2: %lu.%lu%%\r\n", spo2_raw_int / 10, spo2_raw_int % 10);

        // 5. 适配新的变长协议进行打包（使用原始的 hr 和 spo2）
        uint32_t hr_X100   = (uint32_t)(hr * 100);
        uint32_t spo2_X100 = (uint32_t)(spo2 * 100);

        //发送给天问适配新的变长协议进行打包
        Tianwen_Packet_t tianwen_packet;
        memset(&tianwen_packet, 0x00, sizeof(Tianwen_Packet_t));
        tianwen_packet.id = SENSOR_ID_HEART_RATE_SPO2;
        tianwen_packet.data_len = 4; // 心率(2字节) + 血氧(2字节) = 4字节
        // 手动指定大端序发送（高位在前）
        tianwen_packet.data[0] = (uint8_t)(hr_X100 >> 8);   // 心率高字节
        tianwen_packet.data[1] = (uint8_t)(hr_X100 & 0xFF); // 心率低字节
        tianwen_packet.data[2] = (uint8_t)(spo2_X100 >> 8); // 血氧高字节
        tianwen_packet.data[3] = (uint8_t)(spo2_X100 & 0xFF);// 血氧低字节
        if (xQueueSend(sendtianwenQueue, &tianwen_packet, 0) != pdPASS) {
            printf("[MAX30102 Callback] Failed to send data to Tianwen Queue!\r\n");
        } 
        // 同时发送到 LCD 显示队列 (保持兼容性)
        if (xQueueSend(sensor_data_lcd_queue, &tianwen_packet, 0) != pdPASS) {
            printf("[MAX30102 Callback] Failed to send data to LCD Queue!\r\n");
        }   
        // 同时也压入 RingBuffer 供 USART 任务异步发送 (保持兼容性)
        Packet_t packet; 
        memset(&packet, 0x00, sizeof(Packet_t));

        // 设定新协议字段
        packet.sensor_id = SENSOR_ID_HEART_RATE_SPO2; // 推荐定义一个新的 ID 0x04
        packet.data_len  = 4; // 心率(2字节) + 血氧(2字节) = 4字节

        // 将数据拷贝到 payload 缓冲区 (注意小端序)
        memcpy(&packet.payload[0], &hr_X100, 2);
        memcpy(&packet.payload[2], &spo2_X100, 2);

        // 6. 压入 RingBuffer (由 usart_task 异步取出并由底层计算 CRC/加头尾)
        if(RingBuffer_push(&g_ring_buffer, &packet) == 0xAF) {
#ifdef MAX30102_HANDLER_DEBUG
            printf("[MAX30102 Callback] Data pushed to RingBuffer\r\n");
#endif
        } 
        else {
            printf("[MAX30102 Callback] RingBuffer FULL!\r\n"); 
        }
    } 
    else 
    {
   
        // 手指脱落或无效时清空历史，保证下次测量独立性
        memset(g_hr_history, 0, sizeof(g_hr_history));
        memset(g_spo2_history, 0, sizeof(g_spo2_history));
        g_filter_idx = 0;

        // 自动重试机制
        if (s_hr_retry_count < MAX_HR_RETRIES) 
        {
            s_hr_retry_count++;
            APP_LOG("[MAX30102] reinit begining (%d/%d)\n", s_hr_retry_count, MAX_HR_RETRIES);
            
            // 往控制中枢发命令，要求再测一次！
            if (xSysCmdQueue != NULL) {
                SystemCommand_t retry_cmd = CMD_MEASURE_HR_SPO2;
                xQueueSend(xSysCmdQueue, &retry_cmd, 0); 
            }
        }
        else 
        {
            // 重试次数耗尽
            printf("[MAX30102]  %d times failed, giving up.\n", MAX_HR_RETRIES);
            s_hr_retry_count = 0; // 重置计数器，等待下一次用户主动触发
        }
    }
}

void bsp_max30102_handler_init(void) 
{
    // 1. 组装 OS 接口
    static max_handler_os_instance_t os_if = {
        .pf_os_create_queue = os_queue_create,
        .pf_os_queue_put    = os_queue_put,
        .pf_os_queue_get    = os_queue_get,
        .pf_os_delay_ms     = os_delay
    };

    // 2. 组装输入接口 (将底层驱动 g_max30102 和 OS 接口绑在一起)
    static max_handler_input_instance_t input_if = {
        .max30102_instance   = &g_max30102, 
        .os_handler_instance = &os_if
    };

    // 3. 实例化 Handler (这步会创建 Queue)
    if (max30102_handler_inst(&g_max30102_handler, &input_if) == MAX_HANDLER_OK) 
    {
        
        xTaskCreate(max30102_handler_task, 
                    "MAX_Handler", 
                    BSP_MAX30102_TASK_STK_SIZE, 
                    &g_max30102_handler, 
                    5,  
                    NULL);
        

    } 
    else 
    {
        printf("[System] MAX30102 Handler Init Failed!\r\n");
    }
}