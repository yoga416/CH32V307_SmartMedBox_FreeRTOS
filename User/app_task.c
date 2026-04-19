
#include "app_task.h"
#include "debug.h"
#include "string.h"
#include "sht40.h"
#include "sht40_handler.h"
#include "Middle_ring_buffer.h"
#include "../bsp/usart_wifi_esp/usart_wifi_esp.h"
#include "../bsp/bsp_MLX_90614/bsp_MLX_90614.h"
#include "../bsp/bsp_MLX_90614/bsp_MLX_90614_handler.h"
#include "system.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "algorithm_1.h"
#include "../bsp/bsp_MLX_90614/bsp_MLX_90614_port.h"
#include "../bsp/sht40/sht40_port.h"
#include "../bsp/bsp_max_30102/bsp_max_30102_port.h"
#include "../bsp/bsp_max_30102/bsp_max_30102_handler.h"
#include "../bsp/bsp_max_30102/bsp_max_30102_driver.h"
/* 全局变量定义 */
RingBuffer_t g_ring_buffer = {0};
QueueHandle_t xUART_Queue = NULL;
SemaphoreHandle_t xDMA_Sem = 0;
SemaphoreHandle_t xBufferDataSemaphore = NULL;

/* 任务句柄 */
TaskHandle_t bspSensorTask_Handler;
TaskHandle_t appTask_Handler;
TaskHandle_t usartTask_Handler;
/* 内部私有函数声明 */
static void MAX30102_Key_Init(void);
static void __attribute__((unused)) MAX30102_TestTask(void);
static void SHT40_TestTask(void);
static void __attribute__((unused)) MLX90614_TestTask(void);
static void __attribute__((unused)) bsp_battery_task(void);
static void __attribute__((unused)) sim800l_test_task(void);
void bsp_sensor_task(void *pvParameters);
void usart_task(void *pvParameters);
void app_task(void *pvParameters);


#include "bsp_max_30102_port.h"
void app_task(void *pvParameters)
{
 
    for (;;)
    {
         
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void bsp_sensor_task(void *pvParameters)
{
    (void)pvParameters;

#if (SENSOR_RUN_MODE == SENSOR_MODE_MAX30102)
    printf("[bsp_sensor_task] Run mode: MAX30102\n");
    MAX30102_TestTask();
#elif (SENSOR_RUN_MODE == SENSOR_MODE_SHT40)
    printf("[bsp_sensor_task] Run mode: SHT40\n");
    SHT40_TestTask();
#elif(SENSOR_RUN_MODE == SENSOR_MODE_BATTERY_CHECK)
    printf("[bsp_sensor_task] Run mode: Battery Check\n");
    bsp_battery_task();
#elif (SENSOR_RUN_MODE == SENSOR_MODE_MLX90614)
    printf("[bsp_sensor_task] Run mode: MLX90614\n");
    MLX90614_TestTask();
#elif(SENSOR_RUN_MODE == SENSOR_MODE_SIM800L)
    printf("[bsp_sensor_task] Run mode: SIM800L\n");
    sim800l_test_task();
#else
    printf("[bsp_sensor_task] Run mode: UNKNOWN\n");
    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
#endif
}

void usart_task(void *pvParameters)
{
    Packet_t packet;

    (void)pvParameters;

    for (;;)
    {
        while (RingBuffer_isEmpty(&g_ring_buffer) == 0xAF)
        {
            if (RingBuffer_pop(&g_ring_buffer, &packet) == 0xAF)
            {
                if (USART_WIFI_ESP_Send(&packet) != USART_WIFI_ESP_OK)
                {
                    printf("[usart_task] Failed to send packet to ESP8266\n");
                }
            }
            else
            {
                printf("[usart_task] Failed to pop packet\n");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
#if 1         //sht40_test

/////////////////////////////////////////////////////////////////////////////////////////////
#include "sht40_handler.h"
#include "sht40_port.h"

static void  SHT40_TestTask(void)
{
    //定义了事件
    temp_humi_event_t event;
    SHT40_HANDLER_Status_t RET=SHT40_HANDLER_OK;
    printf("[SHT40_TestTask] task start!\r\n");
    for(;;)
    {
        bsp_temp_humi_handler_t *handler = (bsp_temp_humi_handler_t *)g_temp_humi_handler_instance;
        event.event_type=TEMP_HUMI_EVENT_BOTH;
        event.lifetime=1000;
        event.pf_callback=on_SHT40_DATA_ready;
        if(handler!=NULL&&
           handler->os_interface!=NULL&&
           handler->event_queue_handle!=NULL)
        {
            RET=handler->os_interface->os_queue_put(
                handler->event_queue_handle,
                &event,
                10);
            if(RET!=SHT40_HANDLER_OK)
            {
                printf("[SHT40_TestTask] put queue is failed\r\n");
            }
            else
            {
                printf("[SHT40_TestTask] queue put ok\r\n");
            }
        }
        else
        {
             printf("[SHT40_TestTask] wait handler mount: inst=0x%08lX os_if=0x%08lX queue=0x%08lX\r\n",
                     (unsigned long)(uint32_t)handler,
                     (unsigned long)(uint32_t)(handler ? handler->os_interface : NULL),
                     (unsigned long)(uint32_t)(handler ? handler->event_queue_handle : NULL));
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

}

#endif

#if 1        //mlx90614_test_driver
static void MLX90614_TestTask(void)
{
   //定义了事件
    th_event_t event;
    MLX90614_HANDLER_Status_t RET=MLX90614_HANDLER_OK;
    printf("[MLX90614_TestTask] task start!\r\n");

    for(;;)
    {
        bsp_th_handler_t *handler = (bsp_th_handler_t *)g_handler_instance;
        event.event_type=th_event_body_temp;
        event.lifetime=1000;
        event.pf_callback=MLX90614_Port_OnDataReady;
        if(handler!=NULL&&
           handler->os_handler_instance!=NULL&&
           handler->event_queue_handler!=NULL)
        {
            RET=handler->os_handler_instance->pf_os_queue_put(
                handler->event_queue_handler,
                &event,
                10);
            if(RET!=MLX90614_HANDLER_OK)
            {
                printf("[MLX90614_TestTask] put queue is failed\r\n");
            }
            else
            {
                printf("[MLX90614_TestTask] queue put ok\r\n");
            }
        }
        else
        {
             printf("[MLX90614_TestTask] wait handler mount: inst=0x%08lX os_if=0x%08lX queue=0x%08lX\r\n",
                     (unsigned long)(uint32_t)handler,
                     (unsigned long)(uint32_t)(handler ? handler->os_handler_instance : NULL),
                     (unsigned long)(uint32_t)(handler ? handler->event_queue_handler : NULL));
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

#endif

#if 1      //MAX30102_test_TASK
static void MAX30102_TestTask(void)
{
   max_event_t event;
    printf("[App Task] 智能药盒健康监测任务已启动...\n");
    printf("\n[App] === 准备开始新的健康测量 ===\n");
    printf("[App] 提示：请将手指平稳放置在传感器上...\n");
    // 给系统留一点稳定时间
    vTaskDelay(pdMS_TO_TICKS(4000));    // 4秒后开始测量

    for (;;)
    {
        // 1. 构造事件
        event.event_type       = max_event_calc_hr_spo2; // 触发“采集+计算”流程
        event.lifetime         = 1000;                   // 队列发送超时时间
        event.sample_count     = 500;                    // 采集 500 个点 (约 5 秒数据)
        event.pf_calc_callback = on_hr_spo2_calculated;  // 挂载计算结果回调

        // 3. 发送指令给 Handler
        if (g_max_handler_instance != NULL) 
        {
            MAX30102_HANDLER_Status_t ret = max30102_handler_send_event(g_max_handler_instance, &event);
            
            if (ret == MAX_HANDLER_OK) {
                printf("事件发送成功，正在采集数据...\n");
            } else {
                printf("事件发送失败，错误码: %d\n", ret);
            }
        }
        else {
            printf("错误:MAX30102 处理器未就绪！\n");
        }

        vTaskDelay(pdMS_TO_TICKS(15000)); // 10秒后再次测量，实际使用中可以根据需求调整频率
    }
    // for(;;)
    // {
    //     vTaskDelay(pdMS_TO_TICKS(1000));
    // }
}
#endif

static void sim800l_test_task(void)
{

    uint8_t ok;
    for (;;) {
        bsp_sim800l_init(9600U);
        printf("[SIM800L] probe baud=9600\n");
        ok = SIM800L_SendAT("AT", "OK", 1200);
        printf("[SIM800L] AT@9600: %s\n", ok ? "OK" : "TIMEOUT");
        if (ok == 0U) {
            printf("[SIM800L] no valid response on 9600 baudrate\n");
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

}

#if 1       //battery_check_task
static void bsp_battery_task(void)
{
    
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
#endif

/* ------------------- FreeRTOS 钩子函数 ------------------- */

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    taskDISABLE_INTERRUPTS();
    printf("[Error] Stack Overflow: %s\n", pcTaskName);
    while (1)
    {
    }
}

void vApplicationMallocFailedHook(void)
{
    taskDISABLE_INTERRUPTS();
    printf("[Error] Malloc Failed\n");
    while (1)
    {
    }
}
