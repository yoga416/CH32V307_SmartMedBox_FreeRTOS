#include "app_task.h"
#include "debug.h"
#include "string.h"
#include "Middle_ring_buffer.h"
#include "ch32v30x_iwdg.h"

/* 控制本文件的调试打印：0=关闭，1=开启 */
#ifndef APP_DEBUG_PRINT
#define APP_DEBUG_PRINT 0
#endif

#if APP_DEBUG_PRINT
#define APP_LOG(...)    printf(__VA_ARGS__)
#else
#define APP_LOG(...)    ((void)0)
#endif

// 传感器相关的 handler 和 port
#include "sht40_handler.h"
#include "../bsp/sht40/sht40_port.h"
#include "../bsp/bsp_MLX_90614/bsp_MLX_90614_handler.h"
#include "../bsp/bsp_MLX_90614/bsp_MLX_90614_port.h"
#include "../bsp/bsp_max_30102/bsp_max_30102_handler.h"
#include "../bsp/bsp_max_30102/bsp_max_30102_port.h"
#include "../bsp/bsp_max_30102/bsp_max_30102_reg.h"
#include "../bsp/bsp_a7670c/bsp_a7670c.h"
#include "../bsp/usart_wifi_esp/usart_wifi_esp.h"
#include "../bsp_rtc/bsp_rtc.h"
#include "../bsp/bsp_tianwen/bsp_tianwen_reg.h"
#include "../bsp/bsp_rtc/bsp_rtc.h"      // 解决 BSP_RTC_Init 报错

#include "debug.h"
#include "system.h"
#include "lcd.h"      // 添加 LCD 头文件
#include "touch.h"
#include "ctpiic.h"
#include <stdio.h>
#include <string.h>
#include "lvgl.h"
#include "lv_port_disp.h"
#include "lv_port_indev.h"
#include "lv_port_indev.h"
#include "information.h"
#include "gui_guider.h"
#include "events_init.h"

/* 触摸屏扫描（FT6336 软件 I2C） */
#include "touch.h"


//////////////////////////
RingBuffer_t g_ring_buffer = {0};
QueueHandle_t xUART_Queue = NULL;
SemaphoreHandle_t xDMA_Sem = 0;
SemaphoreHandle_t xBufferDataSemaphore = NULL;
QueueHandle_t xSysCmdQueue = NULL; // 系统命令队列
QueueHandle_t CmdQueue = NULL; // 系统命令队列
QueueHandle_t sendtianwenQueue = NULL; // 天问通信命令队列
QueueHandle_t sensor_data_lcd_queue = NULL; // 传感器数据队列
SemaphoreHandle_t xGuiMutex = NULL; // LVGL 互斥锁，防止多任务并发访问 GUI
SemaphoreHandle_t xSem_face_recog;// 人脸识别检测开始同步信号量
SemaphoreHandle_t xSem_face_result_yes; // 人脸识别结果信号量(识别成功)
SemaphoreHandle_t xSem_face_result_no; // 人脸识别结果信号量(识别失败)
SemaphoreHandle_t xSem_face_add; // 人脸添加开始同步信号量
SemaphoreHandle_t xSem_face_add_result; // 人脸添加结果同步信号量
SemaphoreHandle_t xSem_Btn_Eat; // 吃药按钮按下同步信号量
/* 任务句柄 */
TaskHandle_t bspSensorTask_Handler;
TaskHandle_t appTask_Handler;
TaskHandle_t usartTask_Handler;
TaskHandle_t watchdogTask_Handler;
TaskHandle_t tianwenTask_Handler;
TaskHandle_t lcdTask_Handler; // 如果有 LCD 任务的话


 extern volatile uint8_t a7670c_ready; // 在A7670C初始化完成后置1
 extern SystemCommand_t received_cmd;
 UI_DisplayData_t g_ui_data; // 全局 UI 数据结构实例

void check_medication_time(void); // 声明检查吃药时间的函数
 // 定义吃药时间表
AlarmSche my_meds[MAX_SCHE] = {
      {10, 01},  
      {10, 02}, 
      {10, 03}   
};

void app_task(void *pvParameters)
{
//主函数。逻辑函数，根据系统指令执行相应的操作
    (void)pvParameters;
    APP_LOG("\n[App] Smart Medicine Box Started\n");

    //发送开机语音播报命令到天问模块
    Tianwen_Packet_t tianwen_packet;
    tianwen_packet.id = CMD_TX_BOOT_VOICE; // 定义一个新的指令，比如 CMD_TX_BOOT_VOICE
    tianwen_packet.data_len = 0; // 没有额外数据(命令帧)
    xQueueSend(sendtianwenQueue, &tianwen_packet, portMAX_DELAY); // 确保开机播报命令一定能发送出去

    //设置时间：
    BSP_RTC_ModifyTime(2026, 5, 20, 10, 00, 25); // 设置初始时间为2026年3月2日00:00:00
    //设置闹钟(已经设置就生效，所以放在主循环前面)
    ///BSP_RTC_UpdateNextAlarm(); // 根据 my_meds 数组设置第一个闹钟

    //定义一个变量来跟踪当前检查的吃药时间索引
   AppMsg_t msg;
    for (;;)
    {
       
        BaseType_t ret = xQueueReceive(xAppEventQueue, &msg, pdMS_TO_TICKS(10));

        if (ret == pdPASS) 
        {
            // 收到具体事件，开始处理分支
            switch (msg.event_id) 
            {
                case MSG_FACE_RECOG_START:// 人脸识别开始事件
                    APP_LOG("[App] Event: Face recognition started\n");
                    // 模拟：给系统自己发一个成功事件（实战中这应该由串口中断发）
                    AppMsg_t sim_msg = { .event_id = MSG_FACE_RECOG_SUCCESS };
                    xQueueSend(xAppEventQueue, &sim_msg, 0);
                    break;

                case MSG_FACE_RECOG_SUCCESS:// 人脸识别成功事件
                    APP_LOG("[App] Event: Face recognition Success\n");
                    tianwen_packet.id = CMD_TX_REGISTERED_USER; 
                    xQueueSend(sendtianwenQueue, &tianwen_packet, 0); 
                    break;

                case MSG_FACE_RECOG_FAIL:// 人脸识别失败事件
                    APP_LOG("[App] Event: Face recognition Fail\n");
                    tianwen_packet.id = CMD_TX_UNREGISTERED; 
                    xQueueSend(sendtianwenQueue, &tianwen_packet, 0); 
                    break;

                case MSG_BTN_EAT_PRESSED:// 吃药按钮按下事件
                    APP_LOG("[App] Event: User pressed EAT button\n");
                    // 用户按键后，主动触发一次时间检查
                    msg.event_id = MSG_CHECK_MED_TIME;
                    xQueueSend(xAppEventQueue, &msg, 0);
                    break;

                case MSG_CHECK_MED_TIME:// 检查吃药时间事件（定时触发或者按键触发）
                    APP_LOG("[App] Event: Checking Medication Time\n");
                    check_medication_time(); // 调用下面封装的函数
                    break;

                // ... 处理其他事件 ...
                default:
                    break;
            }
        }
        
        /* 喂狗：通知 watchdog_task */
        if (watchdogTask_Handler != NULL) {
            xTaskNotifyGive(watchdogTask_Handler);
        }
    
        vTaskDelay(pdMS_TO_TICKS(20)); // 每20ms检查一次事件队列和喂一次狗
    }
}


void lcd_task(void *pvParameters)
{
    
    for(;;) {
        /* 使用 portMAX_DELAY 阻塞等待，确保拿到锁再执行 LVGL 定时器。
           若用短超时跳过则 lv_timer_handler 无法运行，GUI 刷新停滞。 */

        if(xSemaphoreTake(xGuiMutex, portMAX_DELAY) == pdTRUE) {
            lv_tick_inc(10);      // 先递增 LVGL tick
            lv_timer_handler();  // 处理 LVGL 定时器
            xSemaphoreGive(xGuiMutex);
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // 每10ms更新一次 LCD
    }
}

void sensor_lcd_task(void *pvParameters)
{
    static Tianwen_Packet_t sensor_lcd_packet;
    uint32_t last_rtc_update_tick = 0;
    uint32_t current_tick = 0;

    (void)pvParameters;

    // --- 初始化静态模拟数据并标记需要刷新 ---
    g_ui_data.wifi_status = 'T';
    
    // 初始化漏服记录
    g_ui_data.missed_records[0] = (MissedDoseRecord){1, "2026/05/20", "08:00"};
    g_ui_data.missed_records[1] = (MissedDoseRecord){2, "2026/05/21", "12:30"};
    g_ui_data.missed_records[2] = (MissedDoseRecord){3, "2026/05/22", "19:00"};
    g_ui_data.missed_records[3] = (MissedDoseRecord){4, "2026/05/23", "22:00"};
    
    // 拷贝闹钟时间表 (假设 my_meds 已经在外部定义)
    memcpy(g_ui_data.meds_schedule, my_meds, sizeof(my_meds));

    // 开机强制全屏刷新一次静态数据
    g_ui_data.update_flags |= (UI_FLAG_WIFI | UI_FLAG_RECORDS | UI_FLAG_SCHEDULE);

    for(;;)
    {
        // 1. 触摸屏扫描
        FT6336_Scan();

        // ==========================================
        // 第一部分：数据接收与结构体更新 (不操作 GUI)
        // ==========================================
        
        // 接收传感器数据队列 (等待 20ms)
        if(xQueueReceive(sensor_data_lcd_queue, &sensor_lcd_packet, pdMS_TO_TICKS(20)) == pdPASS) 
        {
            switch(sensor_lcd_packet.id)
            {
                case SENSOR_ID_HEART_RATE_SPO2: 
                    g_ui_data.heart_rate = (sensor_lcd_packet.data[0] << 8) | sensor_lcd_packet.data[1];
                    g_ui_data.spo2       = (sensor_lcd_packet.data[2] << 8) | sensor_lcd_packet.data[3];
                    g_ui_data.update_flags |= UI_FLAG_HR_SPO2; // 标记需要刷新
                    break;

                case SENSOR_ID_MLX_TEMP_BOTH: 
                    g_ui_data.body_temp = (sensor_lcd_packet.data[0] << 8) | sensor_lcd_packet.data[1];
                    g_ui_data.update_flags |= UI_FLAG_BODY_TEMP;
                    break;

                case SENSOR_ID_TEMPERATURE_HUMIDITY: 
                    g_ui_data.env_temp = (sensor_lcd_packet.data[0] << 8) | sensor_lcd_packet.data[1];
                    g_ui_data.env_humi = (sensor_lcd_packet.data[2] << 8) | sensor_lcd_packet.data[3];
                    g_ui_data.update_flags |= UI_FLAG_ENV_TH;
                    break;
            }
        }
        
        // 限制 1秒一次刷新 RTC 时间
        current_tick = xTaskGetTickCount();
        if ((current_tick - last_rtc_update_tick) >= pdMS_TO_TICKS(1000)) 
        {
            last_rtc_update_tick = current_tick;
            BSP_RTC_GetDateTime(&g_ui_data.time);
            g_ui_data.update_flags |= UI_FLAG_TIME;
        }

        // ==========================================
        // 第二部分：统一执行 LVGL UI 渲染
        // ==========================================
        
        // 只有当有数据发生变化时，才去获取互斥锁刷新屏幕
        if (g_ui_data.update_flags != 0) 
        {
            if(xSemaphoreTake(xGuiMutex, pdMS_TO_TICKS(100)) == pdTRUE) 
            {
                // 1. 刷新心率血氧
                if (g_ui_data.update_flags & UI_FLAG_HR_SPO2) {
                    if (lv_obj_is_valid(guider_ui.screen_2_label_13))
                        lv_label_set_text_fmt(guider_ui.screen_2_label_13, "%d.%02d b", g_ui_data.heart_rate/100, g_ui_data.heart_rate%100); 
                    if (lv_obj_is_valid(guider_ui.screen_2_label_14))
                        lv_label_set_text_fmt(guider_ui.screen_2_label_14, "%d.%02d %%", g_ui_data.spo2/100, g_ui_data.spo2%100); 
                }

                // 2. 刷新体温 
                if (g_ui_data.update_flags & UI_FLAG_BODY_TEMP) {
                    if (lv_obj_is_valid(guider_ui.screen_2_label_12)) // 应该使用体温的 label，假设是 12
                        lv_label_set_text_fmt(guider_ui.screen_2_label_12, "%d.%02d°C", g_ui_data.body_temp/100, g_ui_data.body_temp%100);
                }

                // 3. 刷新环境温湿度
                if (g_ui_data.update_flags & UI_FLAG_ENV_TH) {
                    if (lv_obj_is_valid(guider_ui.screen_label_7))
                        lv_label_set_text_fmt(guider_ui.screen_label_7, "%d.%02d°C", g_ui_data.env_temp/100, g_ui_data.env_temp%100);
                    if (lv_obj_is_valid(guider_ui.screen_label_8))
                        lv_label_set_text_fmt(guider_ui.screen_label_8, "%d.%02d %%", g_ui_data.env_humi/100, g_ui_data.env_humi%100);
                }

                // 4. 刷新时间
                if (g_ui_data.update_flags & UI_FLAG_TIME) {
                    if (lv_obj_is_valid(guider_ui.screen_label_4))
                        lv_label_set_text_fmt(guider_ui.screen_label_4, "%02d:%02d:%02d", g_ui_data.time.hour, g_ui_data.time.min, g_ui_data.time.sec);
                    if (lv_obj_is_valid(guider_ui.screen_label_3))
                        lv_label_set_text_fmt(guider_ui.screen_label_3, "%04d/%02d/%02d", g_ui_data.time.year, g_ui_data.time.month, g_ui_data.time.day);
                }

                // 5. 刷新WIFI
                if (g_ui_data.update_flags & UI_FLAG_WIFI) {
                    if (lv_obj_is_valid(guider_ui.screen_label_2))
                        lv_label_set_text_fmt(guider_ui.screen_label_2, "%c", g_ui_data.wifi_status);
                }

                // 6. 刷新漏服记录 (静态数据，只有开机或有新记录时刷新一次)
                if (g_ui_data.update_flags & UI_FLAG_RECORDS) {
                    if(lv_obj_is_valid(guider_ui.screen_2_label_2))
                        lv_label_set_text_fmt(guider_ui.screen_2_label_2,"id:%d date:%s time:%s", g_ui_data.missed_records[0].id, g_ui_data.missed_records[0].date, g_ui_data.missed_records[0].time); 
                    if(lv_obj_is_valid(guider_ui.screen_2_label_4))
                        lv_label_set_text_fmt(guider_ui.screen_2_label_4,"id:%d date:%s time:%s", g_ui_data.missed_records[1].id, g_ui_data.missed_records[1].date, g_ui_data.missed_records[1].time); 
                    if(lv_obj_is_valid(guider_ui.screen_2_label_5))
                        lv_label_set_text_fmt(guider_ui.screen_2_label_5,"id:%d date:%s time:%s", g_ui_data.missed_records[2].id, g_ui_data.missed_records[2].date, g_ui_data.missed_records[2].time); 
                    if(lv_obj_is_valid(guider_ui.screen_2_label_3))
                        lv_label_set_text_fmt(guider_ui.screen_2_label_3,"id:%d date:%s time:%s", g_ui_data.missed_records[3].id, g_ui_data.missed_records[3].date, g_ui_data.missed_records[3].time); 
                }

                // 7. 刷新用药方案
                if (g_ui_data.update_flags & UI_FLAG_SCHEDULE) {
                    if(lv_obj_is_valid(guider_ui.screen_4_label_2))
                        lv_label_set_text_fmt(guider_ui.screen_4_label_2, "Time: %02d:%02d", g_ui_data.meds_schedule[0].hour, g_ui_data.meds_schedule[0].min);
                    if(lv_obj_is_valid(guider_ui.screen_4_label_20))
                        lv_label_set_text_fmt(guider_ui.screen_4_label_20, "Time: %02d:%02d", g_ui_data.meds_schedule[1].hour, g_ui_data.meds_schedule[1].min);
                    if(lv_obj_is_valid(guider_ui.screen_4_label_23))
                        lv_label_set_text_fmt(guider_ui.screen_4_label_23, "Time: %02d:%02d", g_ui_data.meds_schedule[2].hour, g_ui_data.meds_schedule[2].min);
                }

                // 刷新完毕，清除所有脏标志位
                g_ui_data.update_flags = 0; 

                xSemaphoreGive(xGuiMutex);
            }
        }
        
        // 适当缩减循环延时，让触摸和数据读取更顺畅 (由于渲染移入了标志位判断，不再浪费CPU)
        vTaskDelay(pdMS_TO_TICKS(50));
    }   
}


void bsp_sensor_task(void *pvParameters)
{
    A7670C_Init();
    for(;;)
    {
        // 阻塞等待系统指令（语音触发）
        if (xQueueReceive(xSysCmdQueue, &received_cmd, portMAX_DELAY) == pdPASS)
        {
            switch (received_cmd)
            {
                case CMD_MEASURE_HR_SPO2:
                    APP_LOG("[App] Received CMD: Measure Heart Rate & SpO2\n");
                    if (g_max_handler_instance != NULL) {
                        max_event_t max_evt;
                        max_evt.event_type       = max_event_calc_hr_spo2;
                        max_evt.lifetime         = 500;
                        max_evt.sample_count     = MAX30102_DATA_NUM;
                        max_evt.pf_calc_callback = on_hr_spo2_calculated;
                        max30102_handler_send_event(g_max_handler_instance, &max_evt);
                    }
                    else {
                        printf("[App] MAX30102 Handler not ready!\n");
                    }
                    break;

                case CMD_MEASURE_BODY_TEMP:
                    APP_LOG("[App] Received CMD: Measure Body Temperature\n");
                    if (g_handler_instance != NULL) {
                        th_event_t mlx_evt;
                        mlx_evt.event_type = th_event_body_temp;
                        mlx_evt.lifetime = 500;
                        mlx_evt.pf_callback = MLX90614_Port_OnDataReady;
                        bsp_th_handler_t *h_mlx = (bsp_th_handler_t *)g_handler_instance;
                        h_mlx->os_handler_instance->pf_os_queue_put(h_mlx->event_queue_handler, &mlx_evt, 10);
                    }
                    break;

                case CMD_MEASURE_ENV_TEMP_HUMI:
                    //APP_LOG("[App] Received CMD: Measure Env Temp & Humidity\n");
                    if (g_temp_humi_handler_instance != NULL) {
                        temp_humi_event_t sht_evt;
                        sht_evt.event_type = TEMP_HUMI_EVENT_BOTH;
                        sht_evt.lifetime = 500;
                        sht_evt.pf_callback = on_SHT40_DATA_ready;
                        bsp_temp_humi_handler_t *h_sht = (bsp_temp_humi_handler_t *)g_temp_humi_handler_instance;
                        h_sht->os_interface->os_queue_put(h_sht->event_queue_handle, &sht_evt, 10);
                    }
                    break;

                case CMD_SEND_SMS_ALERT:
                    APP_LOG("[App] Received CMD: Send SMS Alert\n");
                    if (!a7670c_ready) {
                        printf("[App] A7670C not ready, cannot send SMS!\n");
                        break;
                    }
                    A7670C_WakeUp();
                    APP_LOG("--- Alert triggered: Sending SMS ---\r\n");
                    A7670C_SendSMS_Auto("18135183446", "请按时服药！");
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    A7670C_EnterSleep();
                    APP_LOG("--- SMS sent, module sleeping ---\r\n");
                    break;

                default:
                    break;
            }
        }
    }
}

/* -------------------------------------------------------------
 * 3. WIFI 数据上传任务
 * ------------------------------------------------------------- */
void usart_task(void *pvParameters)
{
    Packet_t packet;
    (void)pvParameters;
    for (;;)
    {
        while (RingBuffer_isEmpty(&g_ring_buffer) == 0x00)
        {
            if (RingBuffer_pop(&g_ring_buffer, &packet) == 0xAF)
            {
                if (USART_WIFI_ESP_Send(&packet) != USART_WIFI_ESP_OK) {
                    printf("[usart_task] Failed to send packet to ESP8266\n");
                }
            }
            /* ⚠️ 关键：内层循环必须延时让低优先级任务（app_task）有机会喂狗 */
            vTaskDelay(pdMS_TO_TICKS(5));
            /* 同时主动喂狗，防止 app_task 被长时间阻塞 */
            if (watchdogTask_Handler != NULL) {
                xTaskNotifyGive(watchdogTask_Handler);
            }
        }
         /* 喂狗：通知 watchdog_task */
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}


void vApplicationMallocFailedHook(void)
{
    taskDISABLE_INTERRUPTS();
    APP_LOG("[Error] Malloc Failed\n");
    while (1){}
}

/* 看门狗喂狗任务：定期喂独立看门狗，防止系统卡死 */
void watchdog_task(void *pvParameters)
{
    (void)pvParameters;
    for(;;)
    {
        // 等待 app_task 的喂狗通知，如果在 2000ms 内等不到，说明 app_task 卡死了
        // 注意：不等待通知直接喂狗掩盖了死机，这里我们恢复通知等待机制
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2000))) 
        {
            IWDG_ReloadCounter();
        } 
        else 
        {
            // 超时未收到通知，可能是卡死了，不喂狗，让系统复位
            APP_LOG("[Watchdog] Timeout waiting for App Task notification!\r\n");
        }
         //printf("the process is running!\n");
            vTaskDelay(pdMS_TO_TICKS(1000));
    }

}


//// 检查吃药时间的函数，封装成一个独立函数，方便在多个地方调用（定时检查和用户按键检查）
void check_medication_time(void) {
    RTC_TimeTypeDef current_time;
    BSP_RTC_GetDateTime(&current_time);
    
    int16_t curr_total_mins = current_time.hour * 60 + current_time.min;

    for (int i = 0; i < MAX_SCHE; i++) {
        int16_t target_total_mins = my_meds[i].hour * 60 + my_meds[i].min;
        int16_t diff_mins = curr_total_mins - target_total_mins;

        Tianwen_Packet_t tw_pkt;
        tw_pkt.data_len = 0;

        if (diff_mins >= -10 && diff_mins <= 10) {
            tw_pkt.id = CMD_TX_TIME_TO_EAT; 
            xQueueSend(sendtianwenQueue, &tw_pkt, 0); 
            break; 
        } else if (diff_mins < -10) {
            tw_pkt.id = CMD_TX_NOT_TIME_TO_EAT; 
            xQueueSend(sendtianwenQueue, &tw_pkt, 0); 
            break; 
        }
    }
}

