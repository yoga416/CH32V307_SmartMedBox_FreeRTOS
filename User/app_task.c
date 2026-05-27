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
 extern AlarmSche my_meds[3]; // 从 bsp_rtc.h 中 extern 引入吃药时间表
 UI_DisplayData_t g_ui_data; // 全局 UI 数据结构实例

void check_medication_time(void); // 声明检查吃药时间的函数

void app_task(void *pvParameters)
{
//主函数。逻辑函数，根据系统指令执行相应的操作
    (void)pvParameters;
    APP_LOG("\n[App] Smart Medicine Box Started\n");
    // 初始化漏服记录

    //发送开机语音播报命令到天问模块
    Tianwen_Packet_t tianwen_packet;
    tianwen_packet.id = CMD_TX_BOOT_VOICE; // 定义一个新的指令，比如 CMD_TX_BOOT_VOICE
    tianwen_packet.data_len = 0; // 没有额外数据(命令帧)
    xQueueSend(sendtianwenQueue, &tianwen_packet, portMAX_DELAY); // 确保开机播报命令一定能发送出去

    //设置时间：
    BSP_RTC_ModifyTime(2026, 5, 20, 23, 59, 55); // 设置初始时间为2026年3月2日23:59:00
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

        //检查是否漏服

    
        vTaskDelay(pdMS_TO_TICKS(20)); // 每20ms检查一次事件队列和喂一次狗
    }
}


void lcd_task(void *pvParameters)
{
    
    for(;;) {

        if(xSemaphoreTake(xGuiMutex, portMAX_DELAY) == pdTRUE) {
            lv_tick_inc(10);      // 先递增 LVGL tick
            lv_timer_handler();  // 处理 LVGL 定时器
            xSemaphoreGive(xGuiMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(5)); // 每20ms更新一次 LCD
    }
}

void sensor_lcd_task(void *pvParameters)
{
    static Tianwen_Packet_t sensor_lcd_packet;
    uint32_t last_rtc_update_tick = 0;
    uint32_t current_tick = 0;
    static uint32_t last_miss_check_tick = 0;  // 漏服检查计时
     (void)pvParameters;

    // 开机初始化：从 Flash 恢复之前保存的历史记录和配置数据
    SystemData_Init_Load();

    // 将 my_meds 同步到 g_ui_data.meds_schedule
    memcpy(g_ui_data.meds_schedule, my_meds, sizeof(my_meds));

    for(;;)
    {
        // 1. 触摸屏扫描
        FT6336_Scan();

        // 每10秒检查一次是否漏服
        current_tick = xTaskGetTickCount();
        if ((current_tick - last_miss_check_tick) >= pdMS_TO_TICKS(10000)) {
            check_missed_doses();
            last_miss_check_tick = current_tick;
        }
        // ==========================================
        // 第一部分：数据接收，追加进入历史数组并自动存 Flash
        // ==========================================
        if(xQueueReceive(sensor_data_lcd_queue, &sensor_lcd_packet, pdMS_TO_TICKS(20)) == pdPASS) 
        {
            switch(sensor_lcd_packet.id)
            {
                case SENSOR_ID_HEART_RATE_SPO2: {
                    uint16_t hr = (sensor_lcd_packet.data[0] << 8) | sensor_lcd_packet.data[1];
                    uint16_t spo2 = (sensor_lcd_packet.data[2] << 8) | sensor_lcd_packet.data[3];
                    // 核心修改：追加到历史数组并存 Flash
                    Add_History_HR_SpO2(hr, spo2); 
                    break;
                }
                case SENSOR_ID_MLX_TEMP_BOTH: {
            // 使用小端模式解析 (低位在前)
            uint16_t bt = (uint16_t)(sensor_lcd_packet.data[0] | (sensor_lcd_packet.data[1] << 8));
    
            // 如果原数据有可能是负数（虽然体温通常为正），建议转为 int16_t
            int16_t bt_signed = (int16_t)bt;
            Add_History_BodyTemp(bt_signed);
            break;
}
                case SENSOR_ID_TEMPERATURE_HUMIDITY: {
                    uint16_t et = (sensor_lcd_packet.data[0] << 8) | sensor_lcd_packet.data[1];
                    uint16_t eh = (sensor_lcd_packet.data[2] << 8) | sensor_lcd_packet.data[3];
                    Add_History_EnvTH(et, eh);
                    break;
                }
            }
        }
        

        // 限制 1秒 更新一次全局系统时间
        current_tick = xTaskGetTickCount();
        if ((current_tick - last_rtc_update_tick) >= pdMS_TO_TICKS(1000)) 
        {
            last_rtc_update_tick = current_tick;
            BSP_RTC_GetDateTime(&g_ui_data.time);
            g_ui_data.update_flags |= UI_FLAG_TIME;
        }

        // ==========================================
        // 第二部分：统一执行 LVGL UI 渲染 (显示数组中最新的数据)
        // ==========================================
        static lv_obj_t *last_scr = NULL;
        lv_obj_t *curr_scr = lv_scr_act();
        
        // 如果页面发生了切换，强制刷新所有数据，防止进入新页面后数据显示为空
        if (curr_scr != last_scr) {
            g_ui_data.update_flags = 0xFFFFFFFF;
            last_scr = curr_scr;
        }

        if (g_ui_data.update_flags != 0) 
        {
            if(xSemaphoreTake(xGuiMutex, pdMS_TO_TICKS(100)) == pdTRUE) 
            {
                // 强制同步：确保 my_meds 始终与 g_ui_data (Flash数据) 一致
                extern AlarmSche my_meds[3];
                memcpy(my_meds, g_ui_data.meds_schedule, sizeof(my_meds));

                // 1. 刷新心率血氧 (读取数组最后一个元素)
                if ((g_ui_data.update_flags & UI_FLAG_HR_SPO2) && g_ui_data.hr_count > 0) {
                    uint16_t latest_hr = g_ui_data.hr_history[g_ui_data.hr_count - 1].hr;
                    uint16_t latest_spo2 = g_ui_data.hr_history[g_ui_data.hr_count - 1].spo2;
                    if (lv_obj_is_valid(guider_ui.screen_2_label_13))
                        lv_label_set_text_fmt(guider_ui.screen_2_label_13, "%d b", latest_hr/100); 
                    if (lv_obj_is_valid(guider_ui.screen_2_label_14))
                        lv_label_set_text_fmt(guider_ui.screen_2_label_14, "%d %%", latest_spo2/100); 
                }

                // 2. 刷新体温 
                if ((g_ui_data.update_flags & UI_FLAG_BODY_TEMP) && g_ui_data.bt_count > 0) {
                    uint16_t latest_bt = g_ui_data.bt_history[g_ui_data.bt_count - 1].body_temp;
                    if (lv_obj_is_valid(guider_ui.screen_2_label_15)) 
                        lv_label_set_text_fmt(guider_ui.screen_2_label_15, "%d.%02d°C", latest_bt/100, latest_bt%100);
                }

                // 3. 刷新环境温湿度
                if ((g_ui_data.update_flags & UI_FLAG_ENV_TH) && g_ui_data.env_count > 0) {
                    uint16_t latest_et = g_ui_data.env_history[g_ui_data.env_count - 1].env_temp;
                    uint16_t latest_eh = g_ui_data.env_history[g_ui_data.env_count - 1].env_humi;
                    if (lv_obj_is_valid(guider_ui.screen_label_7))
                        lv_label_set_text_fmt(guider_ui.screen_label_7, "%d.%02d°C", latest_et/100, latest_et%100);
                    if (lv_obj_is_valid(guider_ui.screen_label_8))
                        lv_label_set_text_fmt(guider_ui.screen_label_8, "%d.%02d %%", latest_eh/100, latest_eh%100);
                }

                 /*显示wifi状态*/
                if (lv_obj_is_valid(guider_ui.screen_label_2))
                {
                    lv_label_set_text_fmt(guider_ui.screen_label_2, "%c", g_ui_data.wifi_status);
                }

                // 4. 刷新时间
                if (g_ui_data.update_flags & UI_FLAG_TIME) {
                    if (lv_obj_is_valid(guider_ui.screen_label_4))
                        lv_label_set_text_fmt(guider_ui.screen_label_4, "%02d:%02d:%02d", 
                            g_ui_data.time.hour, g_ui_data.time.min, g_ui_data.time.sec);
                    if (lv_obj_is_valid(guider_ui.screen_label_3))
                        lv_label_set_text_fmt(guider_ui.screen_label_3, "%04d/%02d/%02d",
                             g_ui_data.time.year, g_ui_data.time.month, g_ui_data.time.day);
                }


                // 6. 刷新漏服记录 — index0=最新，显示4行，第一行最新
                if (g_ui_data.update_flags & UI_FLAG_RECORDS) {
                    lv_obj_t* labels[] = {
                        guider_ui.screen_2_label_2, // 第一行 — 最新记录 (index 0)
                        guider_ui.screen_2_label_4, // 第二行 — 次新 (index 1)
                        guider_ui.screen_2_label_5, // 第三行 (index 2)
                        guider_ui.screen_2_label_3  // 第四行 — 最旧 (index 3)
                    };
                    for(int i = 0; i < 4; i++) {
                        if(lv_obj_is_valid(labels[i])) {
                            if(i < g_ui_data.record_count) {
                                lv_label_set_text_fmt(labels[i], "no:%lu time:%s turn:%s",
                                    (unsigned long)g_ui_data.missed_records[i].id,
                                    g_ui_data.missed_records[i].date,
                                    g_ui_data.missed_records[i].time);
                               
                            } else {
                                lv_label_set_text(labels[i], ""); // 无数据则空白
                            }
                        }
                    }
                }
                
                if(g_ui_data.update_flags & UI_FLAG_SCHEDULE) {
                    if(lv_obj_is_valid(guider_ui.screen_4_label_2))
                        lv_label_set_text_fmt(guider_ui.screen_4_label_2, "Time: %02d:%02d", 
                            g_ui_data.meds_schedule[0].hour, g_ui_data.meds_schedule[0].min);
                    
                    if(lv_obj_is_valid(guider_ui.screen_4_label_20))
                        lv_label_set_text_fmt(guider_ui.screen_4_label_20, "Time: %02d:%02d", 
                            g_ui_data.meds_schedule[1].hour, g_ui_data.meds_schedule[1].min);
                    
                    if(lv_obj_is_valid(guider_ui.screen_4_label_23))
                        lv_label_set_text_fmt(guider_ui.screen_4_label_23, "Time: %02d:%02d", 
                            g_ui_data.meds_schedule[2].hour, g_ui_data.meds_schedule[2].min);


                    /* 3. 同步下拉框选中状态 */
                    if(lv_obj_is_valid(guider_ui.screen_4_ddlist_4)) {
                        uint16_t sel = (g_ui_data.meds_schedule[0].pill_count > 0) ? (g_ui_data.meds_schedule[0].pill_count - 1) : 0;
                        lv_dropdown_set_selected(guider_ui.screen_4_ddlist_4, sel);
                    }
                    if(lv_obj_is_valid(guider_ui.screen_4_ddlist_2)) {
                        uint16_t sel = (g_ui_data.meds_schedule[1].pill_count > 0) ? (g_ui_data.meds_schedule[1].pill_count - 1) : 0;
                        lv_dropdown_set_selected(guider_ui.screen_4_ddlist_2, sel);
                    }
                    if(lv_obj_is_valid(guider_ui.screen_4_ddlist_3)) {
                        uint16_t sel = (g_ui_data.meds_schedule[2].pill_count > 0) ? (g_ui_data.meds_schedule[2].pill_count - 1) : 0;
                        lv_dropdown_set_selected(guider_ui.screen_4_ddlist_3, sel);
                    }
                }
                 // 刷新完成后，清除更新标志
                g_ui_data.update_flags = 0; 
                xSemaphoreGive(xGuiMutex);
            }
        }
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
 * 3. WIFI 数据上传与指令接收任务
 * ------------------------------------------------------------- */
void usart_task(void *pvParameters)
{
    Packet_t packet;
    Packet_t rx_packet;
    static TickType_t last_sync_tick = 0;  // 记录上次同步时间
    static uint8_t is_first_sync = 1;      // 首次同步标志
    static TickType_t last_wifi_tick = 0;  // 记录上次收到WiFi状态的时间
    (void)pvParameters;
    for (;;)
    {
    

        // 1. 发送逻辑：处理 RingBuffer 中的待发送数据
        while (RingBuffer_isEmpty(&g_ring_buffer) == 0x00)
        {
            if (RingBuffer_pop(&g_ring_buffer, &packet) == 0xAF)
            {
                if (USART_WIFI_ESP_Send(&packet) != USART_WIFI_ESP_OK) {
                    printf("[usart_task] Failed to send packet to ESP8266\n");
                }
            }
            vTaskDelay(pdMS_TO_TICKS(5));
            if (watchdogTask_Handler != NULL) {
                xTaskNotifyGive(watchdogTask_Handler);
            }
        }

        // 2. 接收逻辑：从 WiFi 接收指令包 (包含时间同步指令)
        if (USART_WIFI_ESP_Receive(&rx_packet) == USART_WIFI_ESP_OK)
        {

            /*时间同步*/
            if (rx_packet.sensor_id == CMD_RTC_SYNC && rx_packet.data_len >= 6)
            {
                uint8_t year   = rx_packet.payload[0];
                uint8_t month  = rx_packet.payload[1];
                uint8_t day    = rx_packet.payload[2];
                uint8_t hour   = rx_packet.payload[3];
                uint8_t minute = rx_packet.payload[4];
                uint8_t second = rx_packet.payload[5];

                TickType_t current_tick = xTaskGetTickCount();
                
                // 每200秒校准一次误差 (首次或者间隔超过200,000ms)
                if (is_first_sync || (current_tick - last_sync_tick) >= pdMS_TO_TICKS(20000))
                {
                    // printf("\r\n[Time Sync] Calibrating RTC (Interval 200s): 20%02d-%02d-%02d %02d:%02d:%02d\n", 
                    //         year, month, day, hour, minute, second);
                    
                    // 修改 RTC 时间
                    BSP_RTC_ModifyTime(2000 + year, month, day, hour, minute, second);
                    
                    last_sync_tick = current_tick;
                    is_first_sync = 0;

                    // 立即更新 LCD 显示的时间
                    BSP_RTC_GetDateTime(&g_ui_data.time);
                    g_ui_data.update_flags |= UI_FLAG_TIME;
                    
                    printf("[Time Sync] RTC & LCD display calibrated successfully!\n");
                }
            }

            /*位置同步*/
            if (rx_packet.sensor_id == CMD_LOCATION_SYNC && rx_packet.data_len >= 8)
            {
                char city_name[16]={0};
                memcpy(city_name, rx_packet.payload, rx_packet.data_len);
                APP_LOG("\r\n[Location Sync] Received new location: %s\n", city_name);
                /*保存在w25qxx中*/
                //W25Q_WriteData(LOCATION_STORAGE_ADDR, (uint8_t*)city_name, 16);
            }

            /*天气同步*/
            if (rx_packet.sensor_id == CMD_WEATHER_SYNC && rx_packet.data_len >= 2)
            {
                int8_t weather_code= (int8_t)rx_packet.payload[0]; // 天气代码
                // 心知天气官方代码参考
                    switch(weather_code) {
                            case 0: APP_LOG("weather: clear"); break;
                            case 4: APP_LOG("weather: cloudy"); break; // 4 对应多云
                            case 6: APP_LOG("weather: overcast"); break;
                            case 9: APP_LOG("weather: partly cloudy"); break;
                            case 10: APP_LOG("weather: shower"); break;
                            // ... 更多代码
                            default:  break;
                            }
                int8_t temperature = (int8_t)rx_packet.payload[1]; // 温度
                APP_LOG("\r\n[Weather Sync] Received weather update: Temp=%d°C\n", temperature);
                /*可以根据天气代码更新 LCD 上的天气图标，温度可以显示在天气图标旁边*/
                /*保存在w25qxx中*/
                //W25Q_WriteData(LOCATION_STORAGE_ADDR, (uint8_t*)city_name, 16);
            }

            /*wifi状态更新*/
            if (rx_packet.sensor_id == CMD_WIFI_STATUS_UPDATE)
            {
                
                APP_LOG("[WiFi] Status update received: 0x%02X\n", rx_packet.payload[0]);
                if(rx_packet.payload[0] == 0x32) {
                    // 接收到心跳包，说明模块在线，更新 WiFi 状态和最后收到状态的时间戳
                    last_wifi_tick = xTaskGetTickCount();
                    g_ui_data.wifi_status = 'T'; // 'T' 表示 WiFi 已连接
                } else {
                    g_ui_data.wifi_status = 'F'; // 'F' 表示 WiFi 断开
                }
                g_ui_data.update_flags |= UI_FLAG_WIFI; // 触发 UI 刷新
            }
        }
       // ==========================================
        // 3. 独立的心跳超时检测逻辑 (放在接收 if 块的外面)
        // ==========================================
        /* 串口 20s 内没有接收到任何数据时，把 wifi_status 设置为 'F' */
        if(g_ui_data.wifi_status == 'T') 
        {
            if ((xTaskGetTickCount() - last_wifi_tick) >= pdMS_TO_TICKS(20200)) 
            {
                APP_LOG("[WiFi] No status update received for 20s, setting status to 'F'\n");
                g_ui_data.wifi_status = 'F';
                g_ui_data.update_flags |= UI_FLAG_WIFI; // 触发 UI 刷新
            }
        }

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
/*规定时间为10分钟*/
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

/*检查是否漏服的函数，封装成一个独立函数，方便在多个地方调用（定时检查和用户按键检查）*/
void check_missed_dose(void) {


}