/******************************************************************************
 *
 * @file sht40_port.c
 * @brief FreeRTOS porting layer implementation for SHT40 Handler.
 *
 *****************************************************************************/
#include "sht40_port.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "Middle_ring_buffer.h"
#include "debug.h"
#include "app_task.h"
#include "sht40_handler.h"
#include "sht40.h"
#include <string.h>
#include "bsp_tianwen_reg.h"

static th_handler_input_interface_t sht40_config;
static bsp_sht40_driver_t g_sht40_driver;

static void OS_Delay_Ms_Wrapper(uint32_t const ms)
{
   
    vTaskDelay(pdMS_TO_TICKS(ms));
}


static SHT40_HANDLER_Status_t queue_create_op(uint32_t const item_num, uint32_t const item_size, void **const queue_handle)
{
    /* xQueueCreate 返回队列句柄，将其强制转换为 void* 赋给底层 */
    *queue_handle = (void *)xQueueCreate(item_num, item_size);
    
    if (*queue_handle != NULL) {
        return SHT40_HANDLER_OK;
    }
    return SHT40_HANDLER_ERROR;
}

/**
 * @brief OS 队列接收(出队)包装
 */
static SHT40_HANDLER_Status_t queue_get_op(void *const queue_handle, void *const item, uint32_t const timeout_ms)
{
    TickType_t wait_ticks;
    if (timeout_ms == 0xFFFFFFFFUL) {
        wait_ticks = portMAX_DELAY;
    } else {
        wait_ticks = pdMS_TO_TICKS(timeout_ms);
    }

    if (xQueueReceive((QueueHandle_t)queue_handle, item, wait_ticks) == pdTRUE) {
        return SHT40_HANDLER_OK;
    }
    return SHT40_HANDLER_ERROR;
}

static SHT40_HANDLER_Status_t queue_put_op(void *const queue_handle, void const *const item, uint32_t const timeout_ms)
{
    TickType_t wait_ticks = pdMS_TO_TICKS(timeout_ms);

    /* 发送数据到队列后方 */
    if (xQueueSend((QueueHandle_t)queue_handle, item, wait_ticks) == pdTRUE) {
        return SHT40_HANDLER_OK;
    }
    return SHT40_HANDLER_ERROR;
}

/* ========================================================================= */
/* ======================== 暴露的全局接口实例 ============================= */
/* ========================================================================= */

th_handler_os_interface_t g_sht40_freertos_if = {
    .os_delay_ms     = OS_Delay_Ms_Wrapper,
    .os_queue_create = queue_create_op,
    .os_queue_get    = queue_get_op,
    .os_queue_put    = queue_put_op
};

//*********************SHT40 Timebase Interface*********************
static uint32_t SHT40_GetTickCount(void)
{
    return (uint32_t)xTaskGetTickCount();
}

static void SHT40_DelayMs(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

static timebase_interface_t g_sht40_timebase_interface = {
    .pf_get_tick_count = SHT40_GetTickCount,
    .pf_delay_ms = SHT40_DelayMs,
};

/************************ Software I2C Implementation *************************/
static void SHT40_BitDelay(void)
{
    volatile uint32_t i;
    for (i = 0; i < SHT40_SOFT_I2C_DELAY_CYCLES; i++) {
        __NOP();
    }
}

static void IIC_W_SCL(uint8_t level) {
    if (level) GPIO_SetBits(SHT40_SOFT_I2C_SCL_PORT, SHT40_SOFT_I2C_SCL_PIN);
    else       GPIO_ResetBits(SHT40_SOFT_I2C_SCL_PORT, SHT40_SOFT_I2C_SCL_PIN);
    SHT40_BitDelay();
}

static void IIC_W_SDA(uint8_t level) {
    if (level) GPIO_SetBits(SHT40_SOFT_I2C_SDA_PORT, SHT40_SOFT_I2C_SDA_PIN);
    else       GPIO_ResetBits(SHT40_SOFT_I2C_SDA_PORT, SHT40_SOFT_I2C_SDA_PIN);
    SHT40_BitDelay();
}

static uint8_t IIC_R_SDA(void) {
    uint8_t level = GPIO_ReadInputDataBit(SHT40_SOFT_I2C_SDA_PORT, SHT40_SOFT_I2C_SDA_PIN);
    SHT40_BitDelay();
    return level;
}

/* --- 核心修复：增加等待 ACK 超时机制，防止死循环 --- */
SHT40_Status_t SHT40_SoftI2C_waitAck(void *context)
{
    uint32_t timeout = 2000; 
    IIC_W_SDA(1);    
    IIC_W_SCL(1);    
    while(IIC_R_SDA() && --timeout); 
    IIC_W_SCL(0); 
    return (timeout > 0) ? SHT40_OK : SHT40_ERROR;
}

SHT40_Status_t SHT40_SoftI2C_Start(void *context)
{
    IIC_W_SDA(1); IIC_W_SCL(1); IIC_W_SDA(0); IIC_W_SCL(0);
    //printf("[SHT40 Soft I2C] Start condition generated.\n");
    return SHT40_OK;
}

SHT40_Status_t SHT40_SoftI2C_Stop(void *context)
{
    IIC_W_SDA(0); IIC_W_SCL(1); IIC_W_SDA(1);
    return SHT40_OK;
}
SHT40_Status_t SHT40_SoftI2C_Init(void *context)
{
    GPIO_InitTypeDef GPIO_InitStructure={0};

    // 使能实际用到的GPIOA和GPIOC时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC, ENABLE);

    // SCL初始化
    GPIO_InitStructure.GPIO_Pin = SHT40_SOFT_I2C_SCL_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD; // 推荐开漏输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SHT40_SOFT_I2C_SCL_PORT, &GPIO_InitStructure);

    // SDA初始化
    GPIO_InitStructure.GPIO_Pin = SHT40_SOFT_I2C_SDA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD; // 推荐开漏输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SHT40_SOFT_I2C_SDA_PORT, &GPIO_InitStructure);

    IIC_W_SCL(1);
    IIC_W_SDA(1);
    return SHT40_OK;
}

SHT40_Status_t SHT40_SoftI2C_Deinit(void *context)
{
    GPIO_InitTypeDef GPIO_InitStructure={0};
    
    GPIO_InitStructure.GPIO_Pin = SHT40_SOFT_I2C_SCL_PIN | SHT40_SOFT_I2C_SDA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; 
    GPIO_Init(SHT40_SOFT_I2C_SCL_PORT, &GPIO_InitStructure);
    return SHT40_OK;
}

SHT40_Status_t SHT40_SoftI2C_SendAck(void *context)
{
    IIC_W_SDA(0); IIC_W_SCL(1); IIC_W_SCL(0); IIC_W_SDA(1);
    return SHT40_OK;
}

SHT40_Status_t SHT40_SoftI2C_SendNoAck(void *context)
{
    IIC_W_SDA(1); IIC_W_SCL(1); IIC_W_SCL(0); IIC_W_SDA(1);
    return SHT40_OK;
}

SHT40_Status_t SHT40_SoftI2C_SendByte(void *context, uint8_t byte)
{
    for (int i = 0; i < 8; i++) {
        IIC_W_SDA((byte & (0x80 >> i)) != 0); 
        IIC_W_SCL(1); IIC_W_SCL(0);
    }
    return SHT40_OK;
}

SHT40_Status_t SHT40_SoftI2C_ReadByte(void *context, uint8_t *data)
{
    uint8_t byte = 0;
    IIC_W_SDA(1); 
    for (int i = 0; i < 8; i++) {
        IIC_W_SCL(1); 
        byte <<= 1;
        if (IIC_R_SDA()) byte |= 0x01;
        IIC_W_SCL(0); 
    }   
    if(data != NULL) *data = byte;
    return SHT40_OK;
}

void pf_empty_critical(void) { __NOP(); }

//回调函数实现
 void on_SHT40_DATA_ready(float * temp ,float *humi)
{
    int32_t temp_x100 = 0;
    int32_t hum_x100 = 0;

    if (temp != NULL && humi != NULL)
    {
        temp_x100 = (int32_t)(*temp * 100.0f);
        hum_x100 = (int32_t)(*humi * 100.0f);
        
       // APP_LOG("[SHT40 Callback] Data ready callback triggered.\n");
       // APP_LOG("[SHT40 Callback] temp=%ld, hum=%ld\n", (long)temp_x100, (long)hum_x100);
        
        // 发送到天问通信队列
Tianwen_Packet_t tianwen_packet;
tianwen_packet.id = SENSOR_ID_TEMPERATURE_HUMIDITY;
tianwen_packet.data_len = 4; // 我们只需要 4 个有效字节[cite: 1]

// 手动拆分温度 (16位有效数据) 放入 data[0] 和 data[1]
tianwen_packet.data[0] = (uint8_t)(temp_x100 >> 8);   // 温度高8位
tianwen_packet.data[1] = (uint8_t)(temp_x100 & 0xFF); // 温度低8位[cite: 2]

// 手动拆分湿度 (16位有效数据) 放入 data[2] 和 data[3]
tianwen_packet.data[2] = (uint8_t)(hum_x100 >> 8);    // 湿度高8位[cite: 2]
tianwen_packet.data[3] = (uint8_t)(hum_x100 & 0xFF);  // 湿度低8位[cite: 2]

// 发送到天问通信队列
if (xQueueSend(sendtianwenQueue, &tianwen_packet, pdMS_TO_TICKS(100)) != pdTRUE) {
    printf("[ERROR] Failed to send data to sendtianwenQueue!\n");
}
//发送到 LCD 显示队列 (保持兼容性)
if (xQueueSend(sensor_data_lcd_queue, &tianwen_packet, pdMS_TO_TICKS(100)) != pdTRUE) {
    printf("[ERROR] Failed to send data to sensor_data_lcd_queue!\n");
}
// 同时也压入 RingBuffer 供 USART 任务异步发送 (保持兼性)

        //发送到环形缓冲区（esp8266 USART 任务消费）
        Packet_t packet_sht;
        packet_sht.sensor_id = SENSOR_ID_TEMPERATURE_HUMIDITY;
        packet_sht.data_len = sizeof(int32_t) * 2;
        memcpy(packet_sht.payload, &temp_x100, sizeof(int32_t));
        memcpy(packet_sht.payload + sizeof(int32_t), &hum_x100, sizeof(int32_t));   

        /* 压入环形缓冲区交给 USART 任务发送 */
        if(RingBuffer_push(&g_ring_buffer, &packet_sht) != 0xAF) {
            printf("[ERROR] SHT40 RingBuffer FULL, data lost!\n");
        } 
    }
    else
    {
        printf("[ERROR] SHT40 Callback] Data pointer is NULL, read failed.\n");
    }
}

//函数实现和绑定
SHT40_Status_t SHT40_Port_InitDriver(bsp_sht40_driver_t *driver)
{
    static iic_driver_interface_t sht40_iic = {0};
    static timebase_interface_t sht40_timebase = {0};
    static os_timebase_interface_t sht40_os_timebase = {0};
      if(driver == NULL) {
        return SHT40_ERROR;
    }
    sht40_iic.pf_iic_init= SHT40_SoftI2C_Init;
    sht40_iic.pf_iic_deinit = SHT40_SoftI2C_Deinit;
    sht40_iic.pf_iic_start = SHT40_SoftI2C_Start;
    sht40_iic.pf_iic_stop = SHT40_SoftI2C_Stop;
    sht40_iic.pf_iic_wait_ack = SHT40_SoftI2C_waitAck;
    sht40_iic.pf_iic_send_ack = SHT40_SoftI2C_SendAck;
    sht40_iic.pf_iic_send_no_ack = SHT40_SoftI2C_SendNoAck;
    sht40_iic.pf_iic_send_byte = SHT40_SoftI2C_SendByte;
    sht40_iic.pf_iic_read_byte = SHT40_SoftI2C_ReadByte;
    sht40_iic.pf_enter_critical = pf_empty_critical;    
    sht40_iic.pf_exit_critical = pf_empty_critical;


    sht40_timebase.pf_get_tick_count = SHT40_GetTickCount;
    sht40_timebase.pf_delay_ms = SHT40_DelayMs;
    sht40_os_timebase.pf_os_delay_ms = OS_Delay_Ms_Wrapper;

    return SHT40_inst(driver, &sht40_iic, &sht40_os_timebase, &sht40_timebase);
}
//将 Handler 输入接口与底层驱动和 OS 接口绑定，供 Handler 内部调用
 SHT40_HANDLER_Status_t SHT40_Port_BindHandlerInput(th_handler_input_interface_t *input, bsp_sht40_driver_t *driver)
{
    if (input == NULL || driver == NULL) {
        return SHT40_HANDLER_ERROR;
    }
    if (SHT40_Port_InitDriver(driver) != SHT40_OK) {
        return SHT40_HANDLER_ERROR;
    }
   
    input->iic_driver_interfaces = driver->p_iic_interface;
    input->timebase_interface = driver->p_timebase_interface;
    input->os_timebase_interface = driver->p_os_timebase_interface;
    input->os_interface = &g_sht40_freertos_if;
    

    return SHT40_HANDLER_OK;
}


/////////////////////////sht40_handler_start配置函数////////////////////////



void sht40_handler_start(void)
{
       if (SHT40_Port_BindHandlerInput(&sht40_config, &g_sht40_driver) != SHT40_HANDLER_OK)
    {
        printf("[ERROR] SHT40 handler bind failed!\r\n");
        return;
    }
    if (xTaskCreate(temp_humi_handler_thread, "sht40_hdl", BSP_SHT40_TASK_STK_SIZE, &sht40_config, 3, NULL) != pdPASS)
    {
        printf("[ERROR] SHT40 task create failed!\r\n");
        return;
    }
}


