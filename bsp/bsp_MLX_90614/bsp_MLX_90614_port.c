#include "bsp_MLX_90614_port.h"
#include <string.h>
#include "debug.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "app_task.h"

#include "bsp_tianwen.h"
extern RingBuffer_t g_ring_buffer;

static void MLX_BitDelay(void)
{
    // ≈6~8us @144MHz，满足 MLX90614 (Max 100kHz) 的半周期要求 (>= 4.7us)
    for(volatile int i = 0; i < 120; i++); 
}

static void IIC_W_SCL(uint8_t level) {
    if (level) GPIO_SetBits(MLX_SCL_PORT, MLX_SCL_PIN);
    else       GPIO_ResetBits(MLX_SCL_PORT, MLX_SCL_PIN);
}

static void IIC_W_SDA(uint8_t level) {
    if (level) GPIO_SetBits(MLX_SDA_PORT, MLX_SDA_PIN);
    else       GPIO_ResetBits(MLX_SDA_PORT, MLX_SDA_PIN);
}

static uint8_t IIC_R_SDA(void) {
    return GPIO_ReadInputDataBit(MLX_SDA_PORT, MLX_SDA_PIN);
}

static MLX90614_Status_t MLX_Iic_Init(void *context) {
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    MLX_RCC_CMD(MLX_RCC_PORT, ENABLE);

    GPIO_InitStructure.GPIO_Pin = MLX_SCL_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD; // 开漏输出 (外部必须有 4.7K 上拉电阻)
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(MLX_SCL_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = MLX_SDA_PIN;
    GPIO_Init(MLX_SDA_PORT, &GPIO_InitStructure);

    IIC_W_SCL(1);
    IIC_W_SDA(1);
    return MLX90614_OK;
}

static MLX90614_Status_t MLX_Iic_Start(void *context) {
    IIC_W_SDA(1); 
    IIC_W_SCL(1); 
    MLX_BitDelay();
    IIC_W_SDA(0); 
    MLX_BitDelay();
    IIC_W_SCL(0);
    MLX_BitDelay(); // 【修复】：Start 结束后 SCL 在低电平，必须给延时，否则下一步 SDA 改变太快
    return MLX90614_OK;
}

static MLX90614_Status_t MLX_Iic_Stop(void *context) {
    IIC_W_SCL(0); 
    IIC_W_SDA(0); 
    MLX_BitDelay();
    IIC_W_SCL(1); 
    MLX_BitDelay();
    IIC_W_SDA(1);
    MLX_BitDelay(); // 【修复】：释放总线后给一小段空闲时间
    return MLX90614_OK;
}

static MLX90614_Status_t MLX_Iic_WaitAck(void *context) {
    uint8_t ack;
    
    IIC_W_SDA(1); // 主机释放SDA，准备接收从机的应答
    MLX_BitDelay();
    IIC_W_SCL(1); // 时钟拉高
    MLX_BitDelay();
    
    ack = IIC_R_SDA(); // 读线
    
    IIC_W_SCL(0); // 时钟拉低
    MLX_BitDelay(); // 【修复】：读完 ACK 拉低后必须延时，保证 SCL 低电平时长
    
    // 如果 SDA 是低电平 (0)，说明有应答
    return (ack == 0) ? MLX90614_OK : MLX90614_ERROR;
}

static MLX90614_Status_t MLX_Iic_SendAck(void *context) {
    IIC_W_SCL(0);
    IIC_W_SDA(0); // 发送 ACK (拉低)
    MLX_BitDelay();
    IIC_W_SCL(1);
    MLX_BitDelay();
    IIC_W_SCL(0);
    MLX_BitDelay(); // 【修复】：拉低后补上延时
    return MLX90614_OK;
}

static MLX90614_Status_t MLX_Iic_SendNoAck(void *context) {
    IIC_W_SCL(0);
    IIC_W_SDA(1); // 发送 NACK (拉高)
    MLX_BitDelay();
    IIC_W_SCL(1);
    MLX_BitDelay();
    IIC_W_SCL(0);
    MLX_BitDelay(); // 【修复】：拉低后补上延时
    return MLX90614_OK;
}

static MLX90614_Status_t MLX_Iic_SendByte(void *context, const uint8_t data) {
    for (int i = 0; i < 8; i++) {
        IIC_W_SDA((data & (0x80 >> i)) != 0); 
        MLX_BitDelay(); // 等待数据线电平稳定 (Setup Time)
        IIC_W_SCL(1); 
        MLX_BitDelay(); // 维持 SCL 高电平
        IIC_W_SCL(0);
        MLX_BitDelay(); // 【致命修复】：必须有这句！保证 SCL 低电平维持足够时间，否则下一次循环立马改变 SDA 会破坏时序。
    }
    IIC_W_SDA(1); // 释放 SDA 总线
    MLX_BitDelay();
    return MLX90614_OK;
}

static MLX90614_Status_t MLX_Iic_RecvByte(void *context, uint8_t *const data) {
    uint8_t byte = 0;
    IIC_W_SDA(1); // 释放 SDA 准备接收
    MLX_BitDelay();
    
    for (int i = 0; i < 8; i++) {
        IIC_W_SCL(0); 
        MLX_BitDelay(); // 此时 SCL 为低，从机正在把数据放到 SDA 上
        IIC_W_SCL(1); 
        MLX_BitDelay(); // 【致命修复】：必须先延时！等开漏总线的上升沿电平彻底稳定后，再去读取。
        
        byte <<= 1;
        if (IIC_R_SDA()) byte |= 0x01; // 此时读取最准
    }   
    IIC_W_SCL(0);
    MLX_BitDelay(); // 【修复】：拉低 SCL 并延时，为接下来的 ACK/NACK 做准备
    
    if(data != NULL) *data = byte;
    return MLX90614_OK;
}

// ... 后面原有的队列、回调等接口代码不用动 ...

static MLX90614_Status_t MLX_Iic_DeInit(void *context) { return MLX90614_OK; }
static void MLX_Iic_EnterCritical(void) { taskENTER_CRITICAL(); }
static void MLX_Iic_ExitCritical(void) { taskEXIT_CRITICAL(); }

/* ==================== OS 接口 ==================== */
static uint32_t MLX_Timebase_GetCount(void *context) { return (uint32_t)xTaskGetTickCount(); }
static uint32_t MLX_Timebase_GetMs(void *context) { return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS); }
static uint32_t MLX_Timebase_GetUs(void *context) { return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS * 1000U); }
static void MLX_Timebase_Delay(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }
static void MLX_Os_DelayMs(uint32_t const ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }

static MLX90614_Status_t MLX_Os_QueueCreate(uint32_t const item_num, uint32_t const item_size, void **const queue_handle) {
    *queue_handle = (void *)xQueueCreate((UBaseType_t)item_num, (UBaseType_t)item_size);
    return (*queue_handle != NULL) ? MLX90614_OK : MLX90614_ERROR;
}

static MLX90614_Status_t MLX_Os_QueuePut(void *const queue_handle, void const *const item, uint32_t timeout_ms) {
    TickType_t wait_ticks = (timeout_ms == 0xFFFFFFFFUL) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return (xQueueSend((QueueHandle_t)queue_handle, item, wait_ticks) == pdTRUE) ? MLX90614_OK : MLX90614_ERROR;
}

static MLX90614_Status_t MLX_Os_QueueGet(void *const queue_handle, void *const item, uint32_t timeout_ms) {
    TickType_t wait_ticks = (timeout_ms == 0xFFFFFFFFUL) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return (xQueueReceive((QueueHandle_t)queue_handle, item, wait_ticks) == pdTRUE) ? MLX90614_OK : MLX90614_ERROR;
}

/* ==================== 数据回调 ==================== */
void MLX90614_Port_OnDataReady(float *surface_temp, float *body_temp)
{
    int32_t surface_temp_x100 = 0;
    int32_t body_temp_x100 = 0;

    if ((surface_temp == NULL) || (body_temp == NULL)) {
        printf("[ERROR] MLX90614 Callback] Data pointer is NULL\r\n");
        return;
    }
    body_temp_x100 = (int32_t)(*body_temp * 100.0f);
    APP_LOG("[MLX90614] Body=%ld\r\n", (long)body_temp_x100);
   // 发送给天问通信任务
    Tianwen_Packet_t tianwen_packet;
    tianwen_packet.id = SENSOR_ID_MLX_TEMP_BOTH;
    tianwen_packet.data_len = 2; // 4字节表面温度 + 4字节人体温度
    // 【加上这 4 行赋值代码】：把体温数据放进准备发送的包裹里
    // 【加上这 4 行赋值代码】：把体温数据放进准备发送的包裹里
   memcpy(&tianwen_packet.data[0], &body_temp_x100, 2); // 只发送体温的低2字节，节省带宽

    if (xQueueSend(sendtianwenQueue, &tianwen_packet, pdMS_TO_TICKS(100)) != pdTRUE) {
        printf("[MLX90614] Failed to send data to Tianwen queue!\r\n");
    }
    // 同时发送到 LCD 显示队列 (保持兼容性)
    if (xQueueSend(sensor_data_lcd_queue, &tianwen_packet, pdMS_TO_TICKS(100)) != pdTRUE) {
        printf("[MLX90614] Failed to send data to LCD queue!\r\n");
    }
     // 同时也压入 RingBuffer 供 USART 任务异步发送 (保持兼容性)
    Packet_t packet; 
    memset(&packet, 0x00, sizeof(Packet_t));

    packet.sensor_id = SENSOR_ID_MLX_TEMP_BOTH; 
    packet.data_len  = 8; 

    memcpy(&packet.payload[0], &surface_temp_x100, 4);
    memcpy(&packet.payload[4], &body_temp_x100, 4);
    
    if (RingBuffer_push(&g_ring_buffer, &packet) == 0xAF) {
        // pushed ok
    } 
    else {
        printf("[MLX90614] Ring buffer full!\r\n");
    }
}

th_handler_os_instance_t g_mlx90614_freertos_if = {
    .pf_os_delay_ms = MLX_Os_DelayMs,
    .pf_os_create_queue = MLX_Os_QueueCreate,
    .pf_os_queue_put = MLX_Os_QueuePut,
    .pf_os_queue_get = MLX_Os_QueueGet,
};

/* ==================== 驱动绑定与启动 ==================== */
MLX90614_Status_t MLX90614_Port_InitDriver(bsp_mlx90614_driver_t *driver)
{
    static mlx_iic_driver_instance_t mlx_iic = {0};
    static mlx_timebase_interface_t mlx_timebase = {0};
    static mlx_os_timebase_interface_t mlx_os_timebase = {0};

    if (driver == NULL) return MLX90614_ERROR;

    mlx_iic.pf_iic_init = MLX_Iic_Init;
    mlx_iic.pf_iic_deinit = MLX_Iic_DeInit;
    
    // 绑定软 IIC 的指针
    mlx_iic.pf_iic_start = MLX_Iic_Start;
    mlx_iic.pf_iic_stop = MLX_Iic_Stop;
    mlx_iic.pf_iic_wait_ack = MLX_Iic_WaitAck;

    mlx_iic.pf_iic_send_ack = MLX_Iic_SendAck;
    mlx_iic.pfiic_send_no_ack = MLX_Iic_SendNoAck;
    mlx_iic.pf_iic_send_byte = MLX_Iic_SendByte;
    mlx_iic.pf_iic_rec_byte = MLX_Iic_RecvByte;
    mlx_iic.pf_enter_critical = MLX_Iic_EnterCritical;
    mlx_iic.pf_exit_critical = MLX_Iic_ExitCritical;

    mlx_timebase.pf_get_count = MLX_Timebase_GetCount;
    mlx_timebase.pf_get_ms = MLX_Timebase_GetMs;
    mlx_timebase.pf_get_us = MLX_Timebase_GetUs;
    mlx_timebase.pf_delay = MLX_Timebase_Delay;
    mlx_os_timebase.pf_os_delay = MLX_Os_DelayMs;

    return bsp_mlx_90614_inst(driver, &mlx_iic, &mlx_os_timebase, &mlx_timebase);
}

MLX90614_Status_t MLX90614_Port_BindHandlerInput(th_handler_input_instance_t *input, bsp_mlx90614_driver_t *driver)
{
    if ((input == NULL) || (driver == NULL)) return MLX90614_ERROR;
    if (MLX90614_Port_InitDriver(driver) != MLX90614_OK) return MLX90614_ERROR;

    input->mlx90614_instance = driver;
    input->iic_driver_interface = driver->p_iic_instance;
    input->timebase_interface = driver->p_timebase_instance;
    input->os_timebase_interface = driver->p_os_timebase_instance;
    input->os_handler_instance = &g_mlx90614_freertos_if;
    return MLX90614_OK;
}

static th_handler_input_instance_t mlx90614_config;
static bsp_mlx90614_driver_t mlx90614_driver;

void MLX90614_Handler_start(void)
{
    if (MLX90614_Port_BindHandlerInput(&mlx90614_config, &mlx90614_driver) != MLX90614_OK) {
        printf("[g_mlx90614] bind failed!\r\n");
        return;
    }
    if (xTaskCreate(th_handler_task, "mlx90614_hdl", BSP_MLX90614_TASK_STK_SIZE, &mlx90614_config, 3, NULL) != pdPASS) {
        printf("[g_mlx90614] task create failed!\r\n");
    }
}