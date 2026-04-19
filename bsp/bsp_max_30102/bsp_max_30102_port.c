#include "bsp_max_30102_port.h"
#include "ch32v30x.h"
#include "task.h"
#include "queue.h"
#include "bsp_max_30102_handler.h"
#include "debug.h"
#include "Middle_ring_buffer.h"
#include "app_task.h"
bsp_max30102_driver_t g_max30102;
SemaphoreHandle_t xSem_MAX30102_Exti = NULL;

/* ---------------- 硬件 I2C 阻塞写 ---------------- */
static MAX_DRIVER_Status_t hw_i2c_write(void *context, uint16_t dev_addr, uint8_t reg, uint8_t data) {
    while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
    I2C_GenerateSTART(I2C1, ENABLE);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
    I2C_Send7bitAddress(I2C1, dev_addr, I2C_Direction_Transmitter);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    I2C_SendData(I2C1, reg);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    I2C_SendData(I2C1, data);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    I2C_GenerateSTOP(I2C1, ENABLE);
    return MAX_DRIVER_OK;
}

/* ---------------- 硬件 I2C 阻塞单字节读 ---------------- */
static MAX_DRIVER_Status_t hw_i2c_read(void *context, uint16_t dev_addr, uint8_t reg, uint8_t *p_data) {
    while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
    I2C_GenerateSTART(I2C1, ENABLE);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
    I2C_Send7bitAddress(I2C1, dev_addr, I2C_Direction_Transmitter);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    I2C_SendData(I2C1, reg);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    
    I2C_GenerateSTART(I2C1, ENABLE);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
    I2C_Send7bitAddress(I2C1, dev_addr, I2C_Direction_Receiver);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
    
    I2C_AcknowledgeConfig(I2C1, DISABLE); // 单字节读完直接 NACK
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
    *p_data = I2C_ReceiveData(I2C1);
    
    I2C_GenerateSTOP(I2C1, ENABLE);
    I2C_AcknowledgeConfig(I2C1, ENABLE);
    return MAX_DRIVER_OK;
}

/* ---------------- 硬件 I2C 阻塞连续多字节读 ---------------- */
static MAX_DRIVER_Status_t hw_i2c_read_mem(void *context, uint16_t dev_addr, uint8_t reg, uint8_t *p_data, uint16_t size) {
    if(size == 0) return MAX_DRIVER_PARAM;

    while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
    I2C_GenerateSTART(I2C1, ENABLE);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
    I2C_Send7bitAddress(I2C1, dev_addr, I2C_Direction_Transmitter);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    I2C_SendData(I2C1, reg);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    
    I2C_GenerateSTART(I2C1, ENABLE);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
    I2C_Send7bitAddress(I2C1, dev_addr, I2C_Direction_Receiver);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
    
    for(uint16_t i = 0; i < size; i++) {
        if(i == size - 1) {
            // 接收最后一个字节前，发送 NACK
            I2C_AcknowledgeConfig(I2C1, DISABLE);
        }
        while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
        p_data[i] = I2C_ReceiveData(I2C1);
    }
    
    I2C_GenerateSTOP(I2C1, ENABLE);
    I2C_AcknowledgeConfig(I2C1, ENABLE); // 恢复应答机制
    return MAX_DRIVER_OK;
}

// /* ---------------- EXTI 中断：传感器 FIFO 数据就绪 ---------------- */
// void EXTI9_5_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
// void EXTI9_5_IRQHandler(void) {
//     if(EXTI_GetITStatus(EXTI_Line6) != RESET) {
//         BaseType_t xWoken = pdFALSE;
//         if(xSem_MAX30102_Exti) xSemaphoreGiveFromISR(xSem_MAX30102_Exti, &xWoken);
//         EXTI_ClearITPendingBit(EXTI_Line6);
//         portYIELD_FROM_ISR(xWoken);
//     }
// }



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
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

    // I2C 引脚 PB8, PB9 重映射
    GPIO_PinRemapConfig(GPIO_Remap_I2C1, ENABLE); 
    GPIO_InitTypeDef g = { 
                        .GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9, 
                        .GPIO_Mode = GPIO_Mode_AF_OD,
                        .GPIO_Speed = GPIO_Speed_50MHz };
    GPIO_Init(GPIOB, &g);

    I2C_InitTypeDef i = { .I2C_ClockSpeed = 400000, 
                        .I2C_Mode = I2C_Mode_I2C, 
                        .I2C_DutyCycle = I2C_DutyCycle_2, 
                        .I2C_OwnAddress1 = 0x00, 
                        .I2C_Ack = I2C_Ack_Enable, 
                        .I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit };
    I2C_Init(I2C1, &i); 
    I2C_Cmd(I2C1, ENABLE);

    // EXTI PC6 
        g.GPIO_Pin = GPIO_Pin_6; 
        g.GPIO_Mode = GPIO_Mode_IPU; // 必须是 IPU (内部上拉输入); 
        GPIO_Init(GPIOC, &g);
        GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource6);
        EXTI_InitTypeDef e = { 
                                .EXTI_Line = EXTI_Line6, 
                                .EXTI_Mode = EXTI_Mode_Interrupt, 
                                .EXTI_Trigger = EXTI_Trigger_Falling, 
                                .EXTI_LineCmd = ENABLE };
        EXTI_Init(&e);

    // 仅创建 EXTI 信号量
    xSem_MAX30102_Exti = xSemaphoreCreateBinary();

    NVIC_EnableIRQ(EXTI9_5_IRQn);

    // 挂载函数
    static max_hw_interface_t hw_if = {
        .hi2c = I2C1,
        .pf_write_reg = hw_i2c_write,
        .pf_read_reg  = hw_i2c_read,
        .pf_read_mem  = hw_i2c_read_mem
    };
    static max_sys_interface_t sys_if = { .pf_delay_ms = Delay_Ms };

    bsp_max30102_inst(&g_max30102, &hw_if, &sys_if);
}



#include <string.h>

// 定义滑动滤波窗口大小（建议 3 到 5，数值越大越平滑但响应越慢）
#define HR_SPO2_FILTER_SIZE 3

// 静态全局历史数组，用于滤波
static int32_t g_hr_history[HR_SPO2_FILTER_SIZE] = {0};
static float   g_spo2_history[HR_SPO2_FILTER_SIZE] = {0.0f};
static uint8_t g_filter_idx = 0;

// 心率血氧解算结果回调函数
void on_hr_spo2_calculated(int32_t hr, int8_t hr_valid, float spo2, int8_t spo2_valid) 
{
    // 检查解算结果的有效性范围
    const uint32_t hr_max_threshold = 180;  // 人的极限心率一般不超过 180
    const uint32_t hr_min_threshold = 40;   // 正常人不会低于 40

    const float spo2_min_threshold = 50.0f; // 血氧低于 50% 早进 ICU 了，改高点防误判
    const float spo2_max_threshold = 100.0f;

    // 1. 总体有效性判断
    bool is_data_valid = (hr_valid && spo2_valid && 
                          (hr >= hr_min_threshold) && (hr <= hr_max_threshold) && 
                          (spo2 >= spo2_min_threshold) && (spo2 <= spo2_max_threshold));

    if (is_data_valid) 
    {
        // 2. 将有效数据压入滑动窗口进行滤波
        g_hr_history[g_filter_idx] = hr;
        g_spo2_history[g_filter_idx] = spo2;
        g_filter_idx = (g_filter_idx + 1) % HR_SPO2_FILTER_SIZE;

        // 3. 计算窗口内的平均值
        int32_t smoothed_hr = 0;
        float smoothed_spo2 = 0.0f;
        uint8_t valid_count = 0;

        for (int i = 0; i < HR_SPO2_FILTER_SIZE; i++) 
        {
            if (g_hr_history[i] > 0) // 剔除初始的 0 值
            {
                smoothed_hr += g_hr_history[i];
                smoothed_spo2 += g_spo2_history[i];
                valid_count++;
            }
        }
        
        if (valid_count > 0) 
        {
            smoothed_hr /= valid_count;
            smoothed_spo2 /= valid_count;
        }

        // 4. 打印平滑后的数据
#ifdef MAX30102_HANDLER_DEBUG
        uint32_t spo2_val = (uint32_t)(smoothed_spo2 * 10.0f + 0.5f); 
        printf("Smoothed Heart Rate: %ld bpm\r\n", smoothed_hr);
        printf("Smoothed SpO2: %lu.%lu%%\r\n", spo2_val / 10, spo2_val % 10);
#endif

        // 5. 扩大 100 倍打包成整数，方便通信传输
        int32_t hr_X100   = smoothed_hr * 100;
        int32_t spo2_X100 = (int32_t)(smoothed_spo2 * 100);

        // 注意：去掉 static 关键字，在 FreeRTOS 中使用局部变量防止不可重入问题
        Packet_t packet; 
        memset(&packet, 0x00, sizeof(Packet_t));

        packet.head[0] = PACKET_HEAD;
        packet.sensor_num = SENSOR_DATA_SIZE; // 假设定义为 2
        
        packet.sensor_data[0].sensor_id = SENSOR_ID_HEART_RATE;
        packet.sensor_data[0].data = (uint16_t)hr_X100;
        
        packet.sensor_data[1].sensor_id = SENSOR_ID_SPO2;
        packet.sensor_data[1].data = (uint16_t)spo2_X100;
        
        packet.length = sizeof(Packet_t);
        packet.crc = Calculate_CRC(&packet);
        packet.tail[0] = PACKET_TAIL;

        // 6. 压入 buffer 中
        if(RingBuffer_push(&g_ring_buffer, &packet) == 0xAF) {
#ifdef MAX30102_HANDLER_DEBUG
            printf("[MAX30102 Callback] Packet pushed to ring buffer\r\n");
#endif
        } 
        else {
#ifdef MAX30102_HANDLER_DEBUG
            printf("[MAX30102 Callback] Failed to push packet! Buffer FULL?\r\n"); 
#endif  
        }
    } 
    else 
    {
#ifdef MAX30102_HANDLER_DEBUG
        printf("Measurement Invalid or Out of Range. Resetting Filters...\r\n");
#endif
        // 【关键】手指一旦松开或数据无效，立刻清空滑动窗口！
        memset(g_hr_history, 0, sizeof(g_hr_history));
        memset(g_spo2_history, 0, sizeof(g_spo2_history));
        g_filter_idx = 0;
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
                    2048, 
                    &g_max30102_handler, 
                    5,  
                    NULL);
        
        printf("[System] MAX30102 Handler Task Created!\r\n");
    } 
    else 
    {
        printf("[System] MAX30102 Handler Init Failed!\r\n");
    }
}