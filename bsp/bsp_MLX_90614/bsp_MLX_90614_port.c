#include "bsp_MLX_90614_port.h"

#include <string.h>

#include "debug.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

MLX90614_Status_t MLX90614_SoftI2C_Init(void);

extern RingBuffer_t g_ring_buffer;

static uint32_t MLX_Timebase_GetCount(void *context)
{
    (void)context;
    return (uint32_t)xTaskGetTickCount();
}

static uint32_t MLX_Timebase_GetMs(void *context)
{
    (void)context;
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static uint32_t MLX_Timebase_GetUs(void *context)
{
    (void)context;
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS * 1000U);
}

static void MLX_Timebase_Delay(uint32_t ms)
{
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        vTaskDelay(pdMS_TO_TICKS(ms));
    }
    else
    {
        Delay_Ms(ms);
    }
}

static void MLX_Os_DelayMs(uint32_t const ms)
{
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        vTaskDelay(pdMS_TO_TICKS(ms));
    }
    else
    {
        Delay_Ms(ms);
    }
}

static MLX90614_Status_t MLX_Os_QueueCreate(uint32_t const item_num,
                                            uint32_t const item_size,
                                            void **const queue_handle)
{
    if ((queue_handle == NULL) || (item_num == 0U) || (item_size == 0U))
    {
        return MLX90614_ERROR;
    }

    *queue_handle = (void *)xQueueCreate((UBaseType_t)item_num, (UBaseType_t)item_size);
    return (*queue_handle != NULL) ? MLX90614_OK : MLX90614_ERROR;
}

static MLX90614_Status_t MLX_Os_QueuePut(void *const queue_handle,
                                         void const *const item,
                                         uint32_t timeout_ms)
{
    TickType_t wait_ticks;

    if ((queue_handle == NULL) || (item == NULL))
    {
        return MLX90614_ERROR;
    }

    wait_ticks = (timeout_ms == 0xFFFFFFFFUL) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return (xQueueSend((QueueHandle_t)queue_handle, item, wait_ticks) == pdTRUE) ? MLX90614_OK
                                                                                 : MLX90614_ERROR;
}

static MLX90614_Status_t MLX_Os_QueueGet(void *const queue_handle,
                                         void *const item,
                                         uint32_t timeout_ms)
{
    TickType_t wait_ticks;

    if ((queue_handle == NULL) || (item == NULL))
    {
        return MLX90614_ERROR;
    }

    wait_ticks = (timeout_ms == 0xFFFFFFFFUL) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return (xQueueReceive((QueueHandle_t)queue_handle, item, wait_ticks) == pdTRUE) ? MLX90614_OK
                                                                                     : MLX90614_ERROR;
}

static MLX90614_Status_t MLX_Iic_Init(void *context)
{
    (void)context;
    return MLX90614_SoftI2C_Init();
}

static MLX90614_Status_t MLX_Iic_DeInit(void *context)
{
    (void)context;
    return MLX90614_OK;
}

static MLX90614_Status_t MLX_Iic_SendAck(void *context)
{
    (void)context;
    return MLX90614_OK;
}

static MLX90614_Status_t MLX_Iic_SendNoAck(void *context)
{
    (void)context;
    return MLX90614_OK;
}

static MLX90614_Status_t MLX_Iic_SendByte(void *context, const uint8_t data)
{
    (void)context;
    (void)data;
    return MLX90614_OK;
}

static MLX90614_Status_t MLX_Iic_RecvByte(void *context, uint8_t *const data)
{
    (void)context;
    if (data != NULL)
    {
        *data = 0U;
    }
    return MLX90614_OK;
}

static void MLX_Iic_EnterCritical(void)
{
    taskENTER_CRITICAL();
}

static void MLX_Iic_ExitCritical(void)
{
    taskEXIT_CRITICAL();
}

void MLX90614_Port_OnDataReady(float *surface_temp, float *body_temp)
{
    int32_t surface_temp_x100 = 0;
    int32_t body_temp_x100 = 0;
    Packet_t packet_mlx = {0};

    if ((surface_temp == NULL) || (body_temp == NULL))
    {
        printf("[MLX90614 Callback] data pointer is NULL, read failed.\r\n");
        return;
    }

    surface_temp_x100 = (int32_t)(*surface_temp * 100.0f);
    body_temp_x100 = (int32_t)(*body_temp * 100.0f);

    printf("[MLX90614 Callback] surface=%ld, body=%ld\r\n",
           (long)surface_temp_x100,
           (long)body_temp_x100);

    packet_mlx.head[0] = PACKET_HEAD;
    packet_mlx.sensor_num = SENSOR_DATA_SIZE;
    packet_mlx.sensor_data[0].sensor_id = SENSOR_ID_MLX_AMBIENT_TEMP;
    packet_mlx.sensor_data[0].data = (uint16_t)surface_temp_x100;
    packet_mlx.sensor_data[1].sensor_id = SENSOR_ID_MLX_OBJECT_TEMP;
    packet_mlx.sensor_data[1].data = (uint16_t)body_temp_x100;
    packet_mlx.length = sizeof(Packet_t);
    packet_mlx.crc = Calculate_CRC(&packet_mlx);
    packet_mlx.tail[0] = PACKET_TAIL;

    if (RingBuffer_push(&g_ring_buffer, &packet_mlx) == 0xAF)
    {
        printf("[MLX90614 Callback] packet pushed to ring buffer\r\n");
    }
    else
    {
        printf("[MLX90614 Callback] failed to push packet to ring buffer\r\n");
    }
}

th_handler_os_instance_t g_mlx90614_freertos_if = {
    .pf_os_delay_ms = MLX_Os_DelayMs,
    .pf_os_create_queue = MLX_Os_QueueCreate,
    .pf_os_queue_put = MLX_Os_QueuePut,
    .pf_os_queue_get = MLX_Os_QueueGet,
};

MLX90614_Status_t MLX90614_Port_InitDriver(bsp_mlx90614_driver_t *driver)
{
    static mlx_iic_driver_instance_t mlx_iic = {0};
    static mlx_timebase_interface_t mlx_timebase = {0};
    static mlx_os_timebase_interface_t mlx_os_timebase = {0};

    if (driver == NULL)
    {
        return MLX90614_ERROR;
    }

    mlx_iic.pf_iic_init = MLX_Iic_Init;
    mlx_iic.pf_iic_deinit = MLX_Iic_DeInit;
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

MLX90614_Status_t MLX90614_Port_BindHandlerInput(th_handler_input_instance_t *input,
                                                 bsp_mlx90614_driver_t *driver)
{
    if ((input == NULL) || (driver == NULL))
    {
        printf("[MLX90614 Port] Invalid input or driver pointer\r\n");
        return MLX90614_ERROR;
    }

    if (MLX90614_Port_InitDriver(driver) != MLX90614_OK)
    {
        return MLX90614_ERROR;
    }

    input->mlx90614_instance = driver;
    input->iic_driver_interface = driver->p_iic_instance;
    input->timebase_interface = driver->p_timebase_instance;
    input->os_timebase_interface = driver->p_os_timebase_instacne;
    input->os_handler_instance = &g_mlx90614_freertos_if;

    return MLX90614_OK;
}
/*****************************启动配置****************************** */
static th_handler_input_instance_t mlx90614_config;
static bsp_mlx90614_driver_t mlx90614_driver;

#if 1     //mlx90614_handler init
void MLX90614_Handler_start(void)
{
    if (MLX90614_Port_BindHandlerInput(&mlx90614_config, &mlx90614_driver) != MLX90614_OK)
    {
        printf("[g_mlx90614] bind failed!\r\n");
        return;
    }

    if (xTaskCreate(th_handler_task, "mlx90614_hdl", 1024, &mlx90614_config, 4, NULL) != pdPASS)
    {
        printf("[g_mlx90614] task create failed!\r\n");
        return;
    }

    printf("[g_mlx90614] task inst success!\r\n");
}
#endif