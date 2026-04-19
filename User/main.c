/**
* File Name          : main.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2021/06/06
* Description        : Main program body.
*/

#include "debug.h"
#include "FreeRTOS.h"
#include "task.h"
#include "system_config.h"
#include "system.h"
#include "sht40.h"
#include "semphr.h"
#include "queue.h"
#include "../bsp/usart_wifi_esp/usart_wifi_esp.h"
#include "string.h"
#include "Middle_ring_buffer.h"


#include "app_task.h"

// Include the header for the MAX30102 sensor

/* 任务函数声明 */
extern void bsp_sensor_task(void *pvParameters);
extern void usart_task(void *pvParameters);
extern void app_task(void *pvParameters);

////////////////////////////////////////////其他测试任务示例//////////////////////////////////////////////////
#if 0
static void SHT40_TestTask(void)
{
    SHT40_Data_t raw_data;
    SHT40_Status_t sht40_status;
    Packet_t packet_sht40;
    int32_t temp_x100;
    int32_t hum_x100;

    for (;;)
    {
        sht40_status = SHT40_Read(&raw_data);
        if (sht40_status == SHT40_OK)
        {
            temp_x100 = (int32_t)(raw_data.temperature_c * 100.0f);
            hum_x100 = (int32_t)(raw_data.humidity_rh * 100.0f);

            memset(&packet_sht40, 0x00, sizeof(Packet_t));
            packet_sht40.head[0] = PACKET_HEAD;
            packet_sht40.sensor_num = SENSOR_DATA_SIZE;
            packet_sht40.sensor_data[0].sensor_id = SENSOR_ID_TEMPERATURE;
            packet_sht40.sensor_data[0].data = (uint32_t)temp_x100;
            packet_sht40.sensor_data[1].sensor_id = SENSOR_ID_HUMIDITY;
            packet_sht40.sensor_data[1].data = (uint32_t)hum_x100;
            packet_sht40.length = sizeof(Packet_t);
            packet_sht40.crc = Calculate_CRC(&packet_sht40);
            packet_sht40.tail[0] = PACKET_TAIL;
            RingBuffer_push(&g_ring_buffer, &packet_sht40);
            printf("[bsp_sensor_task] SHT40 read success, temp=%ld, hum=%ld\n",
                   (long)temp_x100,
                   (long)hum_x100);
        }
        else
        {
            printf("[bsp_sensor_task] SHT40 read failed\n");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
////////////////////////////MLX90614测试任务示例//////////////////////////////////////////////////
static void MLX90614_TestTask(void)
{
    float ambient_temp_c;
    float object_temp_c;
    MLX90614_Status_t ambient_status;
    MLX90614_Status_t object_status;
    Packet_t packet_mlx;

    ambient_temp_c = 0.0f;
    object_temp_c = 0.0f;

    for (;;)
    {
        ambient_status = MLX90614_ReadAmbientTempC(&ambient_temp_c);
        object_status = MLX90614_ReadObjectTempC(&object_temp_c);

        if ((ambient_status == MLX90614_OK) && (object_status == MLX90614_OK))
        {
            memset(&packet_mlx, 0x00, sizeof(Packet_t));
            packet_mlx.head[0] = PACKET_HEAD;
            packet_mlx.sensor_num = SENSOR_DATA_SIZE;
            packet_mlx.sensor_data[0].sensor_id = SENSOR_ID_MLX_AMBIENT_TEMP;
            packet_mlx.sensor_data[0].data = (uint32_t)(ambient_temp_c * 100.0f);
            packet_mlx.sensor_data[1].sensor_id = SENSOR_ID_MLX_OBJECT_TEMP;
            packet_mlx.sensor_data[1].data = (uint32_t)(object_temp_c * 100.0f);
            packet_mlx.length = sizeof(Packet_t);
            packet_mlx.crc = Calculate_CRC(&packet_mlx);
            packet_mlx.tail[0] = PACKET_TAIL;
            RingBuffer_push(&g_ring_buffer, &packet_mlx);
            printf("[bsp_sensor_task] MLX90614 read success, ta=%ld, to=%ld (x0.01C)\n",
                   (long)packet_mlx.sensor_data[0].data,
                   (long)packet_mlx.sensor_data[1].data);
        }
        else
        {
            printf("[bsp_sensor_task] MLX90614 read failed, ta=%d, to=%d\n", ambient_status, object_status);
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
#endif

// MAX30102测试任务示例，按键触发单次测量，持续10秒，期间采样并保存数据，10秒后分析数据并打印结果
#if 0
void bsp_sensor_task(void *pvParameters)
{
    typedef struct {
        uint32_t heart_rate;
        uint32_t spo2;
    } MAX30102_ResultSample_t;

    MAX_30102_Status_t max_status;
    uint32_t heart_rate = 0;
    uint32_t spo2 = 0;
    Packet_t packet;
    MAX_30102_Status_t ret_status;
    TickType_t max_session_start = 0;
    uint32_t max_last_countdown_sec = 0;
    uint8_t prev_key_pressed = 0U;
    MAX30102_ResultSample_t session_samples[MAX30102_STABLE_SAMPLE_MIN_COUNT + 3U];
    uint8_t session_sample_count = 0U;
    uint8_t key_pressed;
    uint8_t dummy_data[6];
    uint8_t wr_ptr;
    uint8_t rd_ptr;
    int32_t pre_flush;
    int32_t i;
    int32_t j;
    uint32_t elapsed_ms;
    uint32_t remain_ms;
    uint32_t remain_sec;
    TickType_t elapsed_ticks;
    TickType_t last_wait_log_tick = 0;
    TickType_t now_ticks;
    uint32_t hr_sorted[MAX30102_STABLE_SAMPLE_MIN_COUNT + 3U];
    uint32_t spo2_sorted[MAX30102_STABLE_SAMPLE_MIN_COUNT + 3U];
    uint32_t median_hr;
    uint32_t median_spo2;
    uint32_t hr_min;
    uint32_t hr_max;
    uint32_t spo2_min;
    uint32_t spo2_max;
    uint32_t hr_tmp;
    uint32_t spo2_tmp;

    (void)pvParameters;

#if (SENSOR_RUN_MODE == SENSOR_MODE_SHT40)
    printf("[bsp_sensor_task] Run mode: SHT40\n");
    SHT40_TestTask();
    return;
#elif (SENSOR_RUN_MODE == SENSOR_MODE_MLX90614)
    printf("[bsp_sensor_task] Run mode: MLX90614\n");
    MLX90614_TestTask();
    return;
#endif

    ret_status = MAX_30102_Init();
    (void)ret_status;

    MAX30102_Key_Init();
    printf("[bsp_sensor_task] Run mode: MAX30102, press PB6 to start 10s measurement\n");

    while (1)
    {
        key_pressed = (GPIO_ReadInputDataBit(MAX30102_KEY_PORT, MAX30102_KEY_PIN) == Bit_RESET) ? 1U : 0U;

        if ((key_pressed != 0U) && (prev_key_pressed == 0U))
        {
            vTaskDelay(pdMS_TO_TICKS(20));
            if (GPIO_ReadInputDataBit(MAX30102_KEY_PORT, MAX30102_KEY_PIN) != Bit_RESET)
            {
                prev_key_pressed = 0U;
                continue;
            }

            pre_flush = 0;
            for (i = 0; i < 32 && pre_flush < 32; i++)
            {
                if (MAX_30102_ReadReg(REG_FIFO_WR_PTR, &wr_ptr) == MAX_30102_OK &&
                    MAX_30102_ReadReg(REG_FIFO_RD_PTR, &rd_ptr) == MAX_30102_OK)
                {
                    if (wr_ptr == rd_ptr)
                    {
                        break;
                    }
                    if (MAX_30102_ReadBurst_Public(REG_FIFO_DATA, dummy_data, 6) == MAX_30102_OK)
                    {
                        pre_flush++;
                    }
                    else
                    {
                        break;
                    }
                }
            }

            while (GPIO_ReadInputDataBit(MAX30102_KEY_PORT, MAX30102_KEY_PIN) == Bit_RESET)
            {
                vTaskDelay(pdMS_TO_TICKS(10));
            }

            max_session_start = xTaskGetTickCount();
            max_last_countdown_sec = 0;
            session_sample_count = 0U;

            while ((xTaskGetTickCount() - max_session_start) < pdMS_TO_TICKS(MAX30102_FIRST_RESULT_DEADLINE_MS))
            {
                elapsed_ticks = xTaskGetTickCount() - max_session_start;
                elapsed_ms = (uint32_t)elapsed_ticks * (uint32_t)portTICK_PERIOD_MS;
                remain_ms = (elapsed_ms < MAX30102_FIRST_RESULT_DEADLINE_MS)
                          ? (MAX30102_FIRST_RESULT_DEADLINE_MS - elapsed_ms)
                          : 0U;
                remain_sec = (remain_ms + 999U) / 1000U;

                if ((remain_sec > 0U) && (remain_sec != max_last_countdown_sec))
                {
                    printf("[bsp_sensor_task] MAX30102 countdown: %lus\n", (unsigned long)remain_sec);
                    max_last_countdown_sec = remain_sec;
                }

                max_status = MAX_30102_ReadData(&heart_rate, &spo2);
                if ((max_status == MAX_30102_OK) &&
                    (session_sample_count < (uint8_t)(sizeof(session_samples) / sizeof(session_samples[0]))))
                {
                    session_samples[session_sample_count].heart_rate = heart_rate;
                    session_samples[session_sample_count].spo2 = spo2;
                    session_sample_count++;
                }

                vTaskDelay(pdMS_TO_TICKS(5));
            }

            if (session_sample_count >= 3U)
            {
                hr_min = session_samples[0].heart_rate;
                hr_max = session_samples[0].heart_rate;
                spo2_min = session_samples[0].spo2;
                spo2_max = session_samples[0].spo2;

                for (i = 0; i < session_sample_count; i++)
                {
                    hr_sorted[i] = session_samples[i].heart_rate;
                    spo2_sorted[i] = session_samples[i].spo2;

                    if (session_samples[i].heart_rate < hr_min)
                    {
                        hr_min = session_samples[i].heart_rate;
                    }
                    if (session_samples[i].heart_rate > hr_max)
                    {
                        hr_max = session_samples[i].heart_rate;
                    }
                    if (session_samples[i].spo2 < spo2_min)
                    {
                        spo2_min = session_samples[i].spo2;
                    }
                    if (session_samples[i].spo2 > spo2_max)
                    {
                        spo2_max = session_samples[i].spo2;
                    }
                }

                for (i = 0; i + 1U < session_sample_count; i++)
                {
                    for (j = i + 1U; j < session_sample_count; j++)
                    {
                        if (hr_sorted[j] < hr_sorted[i])
                        {
                            hr_tmp = hr_sorted[i];
                            hr_sorted[i] = hr_sorted[j];
                            hr_sorted[j] = hr_tmp;
                        }
                        if (spo2_sorted[j] < spo2_sorted[i])
                        {
                            spo2_tmp = spo2_sorted[i];
                            spo2_sorted[i] = spo2_sorted[j];
                            spo2_sorted[j] = spo2_tmp;
                        }
                    }
                }

                median_hr = hr_sorted[session_sample_count / 2U];
                median_spo2 = spo2_sorted[session_sample_count / 2U];

                if ((median_hr >= MAX30102_UPLOAD_HR_MIN_BPM) && (median_hr <= MAX30102_UPLOAD_HR_MAX_BPM) &&
                    (median_spo2 >= MAX30102_UPLOAD_SPO2_MIN_PCT) && (median_spo2 <= MAX30102_UPLOAD_SPO2_MAX_PCT))
                {
                    memset(&packet, 0x00, sizeof(Packet_t));
                    packet.head[0] = PACKET_HEAD;
                    packet.sensor_num = SENSOR_DATA_SIZE;
                    packet.sensor_data[0].sensor_id = SENSOR_ID_HEART_RATE;
                    packet.sensor_data[0].data = median_hr;
                    packet.sensor_data[1].sensor_id = SENSOR_ID_SPO2;
                    packet.sensor_data[1].data = median_spo2;
                    packet.length = sizeof(Packet_t);
                    packet.crc = Calculate_CRC(&packet);
                    packet.tail[0] = PACKET_TAIL;
                    RingBuffer_push(&g_ring_buffer, &packet);
                    printf(" heart_rate=%lu bpm, spo2=%lu%%\n",
                           (unsigned long)median_hr,
                           (unsigned long)median_spo2);
                }
                else
                {
                    printf("[bsp_sensor_task] MAX30102 library output rejected: median_hr=%lu, median_spo2=%lu, samples=%u, hr_range=%lu..%lu, spo2_range=%lu..%lu\n",
                           (unsigned long)median_hr,
                           (unsigned long)median_spo2,
                           session_sample_count,
                           (unsigned long)hr_min,
                           (unsigned long)hr_max,
                           (unsigned long)spo2_min,
                           (unsigned long)spo2_max);
                }
            }
            else
            {
                printf("[bsp_sensor_task] MAX30102 measurement failed (samples=%u)\n", session_sample_count);
            }
        }
        else
        {
            now_ticks = xTaskGetTickCount();
            if ((now_ticks - last_wait_log_tick) >= pdMS_TO_TICKS(2000))
            {
                printf("[bsp_sensor_task] waiting for PB6 key press...\n");
                last_wait_log_tick = now_ticks;
            }
        }

        prev_key_pressed = (GPIO_ReadInputDataBit(MAX30102_KEY_PORT, MAX30102_KEY_PIN) == Bit_RESET) ? 1U : 0U;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
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

    }
}
#endif


/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    SystemCoreClockUpdate();
    Delay_Init();
    USART_Printf_Init(115200);

//全局外设初始化，包含DMA、传感器接口等
    uint8_t ret = 0;
    ret = g_peripheral_init();
    if (ret != 0x32)
    {
        printf("Peripheral Init Failed!!\n");
    }
    else
    {
        printf("Peripheral Init Success!!\n");
    }

    if (xTaskCreate((TaskFunction_t )app_task,
                        (const char*    )"app_task",
                        (uint16_t       )APP_TASK_STK_SIZE,
                        (void*          )NULL,
                        (UBaseType_t    )APP_TASK_PRIO,
                        (TaskHandle_t*  )&appTask_Handler) != pdPASS)
    {
        printf("[Error] Create app_task failed\n");
        while (1) {}
    }
    if (xTaskCreate((TaskFunction_t )bsp_sensor_task,
                        (const char*    )"bsp_sensor_task",
                        (uint16_t       )SENSOR_TASK_STK_SIZE,
                        (void*          )NULL,
                        (UBaseType_t    )SENSOR_TASK_PRIO,
                        (TaskHandle_t*  )&bspSensorTask_Handler) != pdPASS)
    {
        printf("[Error] Create bsp_sensor_task failed\n");
        while (1) {}
    }
    if (xTaskCreate((TaskFunction_t )usart_task,
                        (const char*    )"usart_task",
                        (uint16_t       )USART_TASK_STK_SIZE,
                        (void*          )NULL,
                        (UBaseType_t    )USART_TASK_PRIO,
                        (TaskHandle_t*  )&usartTask_Handler) != pdPASS)
    {
        printf("[Error] Create usart_task failed\n");
        while (1) {}
    }
    printf("All tasks created successfully, starting scheduler...\n");
    // 在 main 函数或合适位置创建测试任务
    // xTaskCreate(MAX30102_TestTask, "MAX30102Test", 256, NULL, tskIDLE_PRIORITY + 1, NULL);

    vTaskStartScheduler();
    /* 如果程序运行到这里，说明内存绝对爆了，调度器没起来 */
    printf("[FATAL] Scheduler failed! Remaining Heap: %d bytes\n", xPortGetFreeHeapSize());
    while (1)
    {
        printf("shouldn't run at here!!\n");
    }
}