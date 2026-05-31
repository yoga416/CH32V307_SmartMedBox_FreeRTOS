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
#include "facial.h"
#include "facial_reg.h"
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
EventGroupHandle_t xWatchdogEventGroup = NULL; // 看门狗事件组
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


// 2. 建立一二线城市映射表 (全局常量，存放在 Flash 中节省 RAM)
const CityMap city_map[] = {
    // 【一线城市】
    {"北京", "Beijing"}, {"上海", "Shanghai"}, {"广州", "Guangzhou"}, {"深圳", "Shenzhen"},

    // 【新一线城市】
    {"成都", "Chengdu"}, {"重庆", "Chongqing"}, {"杭州", "Hangzhou"}, {"武汉", "Wuhan"},
    {"西安", "Xi'an"},   {"郑州", "Zhengzhou"}, {"青岛", "Qingdao"},  {"长沙", "Changsha"},
    {"天津", "Tianjin"}, {"苏州", "Suzhou"},    {"南京", "Nanjing"},  {"东莞", "Dongguan"},
    {"沈阳", "Shenyang"},{"合肥", "Hefei"},     {"佛山", "Foshan"},

    // 【二线城市】(华北/东北/西北)
    {"太原", "Taiyuan"}, {"石家庄", "Shijiazhuang"}, {"济南", "Jinan"}, {"大连", "Dalian"},
    {"哈尔滨", "Harbin"},{"长春", "Changchun"},   {"兰州", "Lanzhou"}, {"银川", "Yinchuan"},
    {"保定", "Baoding"}, {"廊坊", "Langfang"},    {"烟台", "Yantai"},  {"潍坊", "Weifang"},
    {"临沂", "Linyi"},   {"呼和浩特", "Hohhot"},

    // 【二线城市】(华东)
    {"宁波", "Ningbo"},  {"福州", "Fuzhou"},      {"无锡", "Wuxi"},    {"厦门", "Xiamen"},
    {"温州", "Wenzhou"}, {"泉州", "Quanzhou"},    {"南昌", "Nanchang"},{"金华", "Jinhua"},
    {"常州", "Changzhou"},{"嘉兴", "Jiaxing"},    {"南通", "Nantong"}, {"徐州", "Xuzhou"},
    {"台州", "Taizhou"}, {"绍兴", "Shaoxing"},    {"扬州", "Yangzhou"},{"盐城", "Yancheng"},

    // 【二线城市】(华南/西南)
    {"昆明", "Kunming"}, {"南宁", "Nanning"},     {"贵阳", "Guiyang"}, {"珠海", "Zhuhai"},
    {"中山", "Zhongshan"},{"惠州", "Huizhou"},    {"海口", "Haikou"},  {"三亚", "Sanya"}
};
 extern volatile uint8_t a7670c_ready; // 在A7670C初始化完成后置1
 extern SystemCommand_t received_cmd;
 extern AlarmSche my_meds[3]; // 从 bsp_rtc.h 中 extern 引入吃药时间表
 extern uint8_t g_current_active_user_id; // 当前活跃用户ID
 extern void custom_bind_time_labels(void);  
 UI_DisplayData_t g_ui_data[MAX_USER]; // 全局 UI 数据结构实例
// 在主循环中处理接收
FR_packet_t rx_pkt;
 static TickType_t last_check_mechine_time = 0;

void check_medication_time(void); // 声明检查吃药时间的函数
const char* convert_city_to_english(const char* chinese_name); // 声明城市名转换函数
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
    BSP_RTC_ModifyTime(current_time_virvual[0], current_time_virvual[1], current_time_virvual[2], current_time_virvual[3], current_time_virvual[4], current_time_virvual[5]); // 设置初始时间为2026年3月2日23:59:00
    //设置闹钟(已经设置就生效，所以放在主循环前面)
    ///BSP_RTC_UpdateNextAlarm(); // 根据 my_meds 数组设置第一个闹钟

    //定义一个变量来跟踪当前检查的吃药时间索引
    AppMsg_t msg;
    AppMsg_t msg_FR;
    FR_packet_t msg_K20;
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
                    msg_K20.sensor_id = 0x00; // 模拟一个传感器ID
                    msg_K20.data_len = 3; // 3字节数据
                    msg_K20.payload[0] = 0x01; // 命令：注册(0x00)/识别(0x01)
                    msg_K20.payload[1] = 0x5A; // 结果：注册失败(0x03)/识别失败(0x00)，注册成功(0x00)/识别成功(0x01)
                    msg_K20.payload[2] = 0x5A; // 用户ID 0x00//0x01/0x02...
                    FR_SendPacket((FR_packet_t *)&msg_K20); 
                    break;
               case MSG_FACE_RECOG_SUCCESS:// 人脸识别成功事件
                    /*切换用户前，保存旧用户的数据*/
                    SystemData_Save_To_Flash_ByUser(g_current_active_user_id);
                    /* 更新为新识别到的用户ID */
                    g_current_active_user_id = msg.data[2]; 
                    APP_LOG("[App] Recognized User ID: %d\n", g_current_active_user_id);
                    
                    // 【新增核心代码】：把新用户的吃药时间瞬间同步给后台的闹钟数组！
                    memcpy(my_meds, g_ui_data[g_current_active_user_id].meds_schedule, sizeof(my_meds));
                    
                    // 更新界面显示
                    g_ui_data[g_current_active_user_id].update_flags |= UI_FLAG_USER_STEP;
                    g_ui_data[g_current_active_user_id].update_flags |= UI_FLAG_SCHEDULE; // 【新增】：让屏幕上的 08:30 等数字立即刷新成新用户的
                    
                    APP_LOG("[App] Event: Face recognition Success\n");
                    tianwen_packet.id = CMD_TX_REGISTERED_USER; 
                    xQueueSend(sendtianwenQueue, &tianwen_packet, 0); 
                    break;

                case MSG_FACE_RECOG_FAIL:// 人脸识别失败事件
                    APP_LOG("[App] Event: Face recognition Fail\n");
                    tianwen_packet.id = CMD_TX_UNREGISTERED; 
                    xQueueSend(sendtianwenQueue, &tianwen_packet, 0); 
                    break;

                case MSG_FACE_ADD_FAIL:// 人脸添加失败事件
                    APP_LOG("[App] Event: Face add Fail\n");
                    break;
                case MSG_FACE_ADD_START:// 人脸添加开始事件
                    APP_LOG("[App] Event: Face add Start\n");
                     msg_K20.sensor_id = 0x00; // 模拟一个传感器ID
                    msg_K20.data_len = 3; // 3字节数据
                    msg_K20.payload[0] = 0x00; // 命令：注册(0x00)/识别(0x01)
                    msg_K20.payload[1] = 0x5A; // 结果：注册失败(0x03)/识别失败(0x00)，注册成功(0x00)/识别成功(0x01)
                    msg_K20.payload[2] = 0x5A; // 用户ID 0x00//0x01/0x02...
                    FR_SendPacket((FR_packet_t *)&msg_K20); 
                    break;
                default:
                    break;
            }
        }
        //    // /*测试串口发送5s发一次*/
        // if(xTaskGetTickCount() - last_time >= pdMS_TO_TICKS(5000)) {
        //     last_time = xTaskGetTickCount();
        // FR_packet_t test_pkt;
        // test_pkt.sensor_id = 0x00; // 模拟一个传感器ID
        // test_pkt.data_len = 3; // 模拟3字节数据
        // test_pkt.payload[0] = 0x01; // 模拟命令：注册(0x00)/识别(0x01)
        // test_pkt.payload[1] = 0x01; // 模拟结果：注册失败(0x03)/识别失败(0x00)，注册成功(0x00)/识别成功(0x01)
        // test_pkt.payload[2] = 0x02; // 模拟用户ID 0x00//0x01/0x02...
        // FR_SendPacket((FR_packet_t *)&test_pkt); // 发送到 ESP8266
        // }

        /*接收人脸识别的数据*/
        if (FR_ReceivePacket_Block(&rx_pkt, pdMS_TO_TICKS(100))==0) {
         if ( rx_pkt.data_len >= 1)
         {
            /* payload[0]: 命令 (0：注册，1：识别) */
            /* payload[1]: 识别结果 (0：失败, 1：成功)/注册结果（2：成功，3：失败） */
            /* payload[2]: 用户ID */ 
            if (rx_pkt.payload[0] == 0x00) { // 注册相关
                if (rx_pkt.payload[1] == 0x02) {
                    msg_FR.event_id = MSG_FACE_ADD_SUCCESS;
                    xQueueSend(xAppEventQueue, &msg_FR, 0);
                    // 可以在这里更新 UI 或者执行其他逻辑
                } else if (rx_pkt.payload[1] == 0x03) {
                    msg_FR.event_id = MSG_FACE_ADD_FAIL;
                    xQueueSend(xAppEventQueue, &msg_FR, 0);
                }
            } else if (rx_pkt.payload[0] == 0x01) { // 识别相关
                if (rx_pkt.payload[1] == 0x01) {
                    msg_FR.event_id = MSG_FACE_RECOG_SUCCESS;
                    msg_FR.data[2] = rx_pkt.payload[2]; 
                    xQueueSend(xAppEventQueue, &msg_FR, 0);
                    // 可以在这里更新 UI 或者执行其他逻辑
                } else if (rx_pkt.payload[1] == 0x00) {
                    msg_FR.event_id = MSG_FACE_RECOG_FAIL;
                    xQueueSend(xAppEventQueue, &msg_FR, 0);
                }
            }   
         }
        }

        //设置看门狗事件位，表示 app_task 正常运行
        xEventGroupSetBits(xWatchdogEventGroup, WDOG_BIT_APP_TASK);
        vTaskDelay(pdMS_TO_TICKS(5)); // 每20ms检查一次事件队列和喂一次狗
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
        //设置看门狗事件位，表示 lcd_task 正常运行
        xEventGroupSetBits(xWatchdogEventGroup, WDOG_BIT_LCD_TASK);
        vTaskDelay(pdMS_TO_TICKS(5)); // 每5ms更新一次 LCD
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

    // 将从 Flash 加载的 meds_schedule 同步到 my_meds
    memcpy(my_meds, g_ui_data[g_current_active_user_id].meds_schedule, sizeof(my_meds));

    for(;;)
    {
        // 1. 触摸屏扫描
        FT6336_Scan();

        // 每10秒检查一次是否漏服
        current_tick = xTaskGetTickCount();
        if ((current_tick - last_miss_check_tick) >= pdMS_TO_TICKS(1000)) {
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
            BSP_RTC_GetDateTime(&g_ui_data[g_current_active_user_id].time);
            g_ui_data[g_current_active_user_id].update_flags |= UI_FLAG_TIME;
        }

        // ==========================================
        // 第二部分：统一执行 LVGL UI 渲染 (显示数组中最新的数据)
        // ==========================================
        static lv_obj_t *last_scr = NULL;
        lv_obj_t *curr_scr = lv_scr_act();
        
        // 缓存当前用户 ID，防止多任务并发修改导致索引混乱
        uint8_t uid = g_current_active_user_id;
        if (uid >= MAX_USER) uid = 0; // 安全兜底

        // 如果页面发生了切换，强制刷新所有数据，防止进入新页面后数据显示为空
        if (curr_scr != last_scr) {
            g_ui_data[uid].update_flags = 0xFFFFFFFF;
            last_scr = curr_scr;
        }

        if (g_ui_data[uid].update_flags != 0) 
        {
            if(xSemaphoreTake(xGuiMutex, pdMS_TO_TICKS(100)) == pdTRUE) 
            {
            
                // 强制同步：确保 my_meds 始终与 g_ui_data (Flash数据) 一致
                extern AlarmSche my_meds[3];
                memcpy(my_meds, g_ui_data[uid].meds_schedule, sizeof(my_meds));

                // 1. 刷新心率血氧 (读取数组最后一个元素)
                if ((g_ui_data[uid].update_flags & UI_FLAG_HR_SPO2) && g_ui_data[uid].hr_count > 0) {
                    uint16_t count = g_ui_data[uid].hr_count;
                    if (count > 0 && count <= MAX_HISTORY) {
                        uint16_t latest_hr = g_ui_data[uid].hr_history[count - 1].hr;
                        uint16_t latest_spo2 = g_ui_data[uid].hr_history[count - 1].spo2;
                        if (lv_obj_is_valid(guider_ui.screen_2_label_13))
                            lv_label_set_text_fmt(guider_ui.screen_2_label_13, "%d bpm", latest_hr/100); 
                        if (lv_obj_is_valid(guider_ui.screen_2_label_14))
                            lv_label_set_text_fmt(guider_ui.screen_2_label_14, "%d %%", latest_spo2/100); 
                    }
                }

                // 2. 刷新体温 
                if ((g_ui_data[uid].update_flags & UI_FLAG_BODY_TEMP) && g_ui_data[uid].bt_count > 0) {
                    uint16_t count = g_ui_data[uid].bt_count;
                    if (count > 0 && count <= MAX_HISTORY) {
                        uint16_t latest_bt = g_ui_data[uid].bt_history[count - 1].body_temp;
                        if (lv_obj_is_valid(guider_ui.screen_2_label_15)) 
                            lv_label_set_text_fmt(guider_ui.screen_2_label_15, "%d.%02d°C", latest_bt/100, latest_bt%100);
                    }
                }

                // 3. 刷新环境温湿度
                if ((g_ui_data[uid].update_flags & UI_FLAG_ENV_TH) && g_ui_data[uid].env_count > 0) {
                    uint16_t count = g_ui_data[uid].env_count;
                    if (count > 0 && count <= MAX_HISTORY) {
                        uint16_t latest_et = g_ui_data[uid].env_history[count - 1].env_temp;
                        uint16_t latest_eh = g_ui_data[uid].env_history[count - 1].env_humi;
                        if (lv_obj_is_valid(guider_ui.screen_label_7))
                            lv_label_set_text_fmt(guider_ui.screen_label_7, "%d.%02d°C", latest_et/100, latest_et%100);
                        if (lv_obj_is_valid(guider_ui.screen_label_8))
                            lv_label_set_text_fmt(guider_ui.screen_label_8, "%d.%02d %%", latest_eh/100, latest_eh%100);
                    }
                }

                 /*显示wifi状态：T → "ON", F → "OFF"*/
                if (lv_obj_is_valid(guider_ui.screen_label_2))
                {
                    if (g_ui_data[uid].wifi_status == 0)
                        lv_label_set_text(guider_ui.screen_label_2, "ON");
                    else if (g_ui_data[uid].wifi_status == 1)
                        lv_label_set_text(guider_ui.screen_label_2, "OFF");
                }

                // 4. 刷新时间
                if (g_ui_data[uid].update_flags & UI_FLAG_TIME) {
                    if (lv_obj_is_valid(guider_ui.screen_label_4))
                        lv_label_set_text_fmt(guider_ui.screen_label_4, "%02d:%02d:%02d", 
                            g_ui_data[uid].time.hour, g_ui_data[uid].time.min, g_ui_data[uid].time.sec);
                    if (lv_obj_is_valid(guider_ui.screen_label_3))
                        lv_label_set_text_fmt(guider_ui.screen_label_3, "%04d/%02d/%02d",
                             g_ui_data[uid].time.year, g_ui_data[uid].time.month, g_ui_data[uid].time.day);
                }


                // 6. 刷新漏服记录 — index0=最新，显示4行，第一行最新
                if (g_ui_data[uid].update_flags & UI_FLAG_RECORDS) {
                    lv_obj_t* labels[] = {
                        guider_ui.screen_2_label_2, // 第一行 — 最新记录 (index 0)
                        guider_ui.screen_2_label_4, // 第二行 — 次新 (index 1)
                        guider_ui.screen_2_label_5, // 第三行 (index 2)
                        guider_ui.screen_2_label_3  // 第四行 — 最旧 (index 3)
                    };
                    for(int i = 0; i < 4; i++) {
                        if(lv_obj_is_valid(labels[i])) {
                            if(i < g_ui_data[uid].record_count) {
                                lv_label_set_text_fmt(labels[i], "tag:%lu day:%s time:%s",
                                    (unsigned long)g_ui_data[uid].missed_records[i].id,
                                    g_ui_data[uid].missed_records[i].date,
                                    g_ui_data[uid].missed_records[i].time);
                               
                            } else {
                                lv_label_set_text(labels[i], ""); // 无数据则空白
                            }
                        }
                    }
                }
                
                if(g_ui_data[uid].update_flags & UI_FLAG_SCHEDULE) {
                    custom_bind_time_labels();
                    if(lv_obj_is_valid(guider_ui.screen_4_label_2))
                        lv_label_set_text_fmt(guider_ui.screen_4_label_2, "Time: %02d:%02d", 
                            g_ui_data[uid].meds_schedule[0].hour, g_ui_data[uid].meds_schedule[0].min);
                    
                    if(lv_obj_is_valid(guider_ui.screen_4_label_20))
                        lv_label_set_text_fmt(guider_ui.screen_4_label_20, "Time: %02d:%02d", 
                            g_ui_data[uid].meds_schedule[1].hour, g_ui_data[uid].meds_schedule[1].min);
                    
                    if(lv_obj_is_valid(guider_ui.screen_4_label_23))
                        lv_label_set_text_fmt(guider_ui.screen_4_label_23, "Time: %02d:%02d", 
                            g_ui_data[uid].meds_schedule[2].hour, g_ui_data[uid].meds_schedule[2].min);


                    /* 3. 同步下拉框选中状态 */
                    if(lv_obj_is_valid(guider_ui.screen_4_ddlist_4)) {
                        uint16_t sel = (g_ui_data[uid].meds_schedule[0].pill_count > 0) ? (g_ui_data[uid].meds_schedule[0].pill_count - 1) : 0;
                        lv_dropdown_set_selected(guider_ui.screen_4_ddlist_4, sel);
                    }
                    if(lv_obj_is_valid(guider_ui.screen_4_ddlist_2)) {
                        uint16_t sel = (g_ui_data[uid].meds_schedule[1].pill_count > 0) ? (g_ui_data[uid].meds_schedule[1].pill_count - 1) : 0;
                        lv_dropdown_set_selected(guider_ui.screen_4_ddlist_2, sel);
                    }
                    if(lv_obj_is_valid(guider_ui.screen_4_ddlist_3)) {
                        uint16_t sel = (g_ui_data[uid].meds_schedule[2].pill_count > 0) ? (g_ui_data[uid].meds_schedule[2].pill_count - 1) : 0;
                        lv_dropdown_set_selected(guider_ui.screen_4_ddlist_3, sel);
                    }
                }

                /*检查是否到吃药时间刷新*/
                if(g_ui_data[uid].update_flags & UI_FLAG_MED_STATUS) 
                {
                if(g_ui_data[uid].screen_3_update_step==true)
                {
                // 5. 刷新吃药状态按钮 (控制 Screen 3 上的按键显示)
                if (lv_obj_is_valid(guider_ui.screen_3_btn_med1)) {
                    if (g_ui_data[uid].med_status[0] == 1) { // 已经吃药
                        if (lv_obj_is_valid(guider_ui.screen_3_btn_med1_label)) 
                            lv_label_set_text(guider_ui.screen_3_btn_med1_label, "已服");
                        lv_obj_add_state(guider_ui.screen_3_btn_med1, LV_STATE_DISABLED); // 禁用按钮
                    } else { // 未吃药
                        if (lv_obj_is_valid(guider_ui.screen_3_btn_med1_label)) 
                            lv_label_set_text(guider_ui.screen_3_btn_med1_label, "吃药");
                        lv_obj_clear_state(guider_ui.screen_3_btn_med1, LV_STATE_DISABLED); // 解除禁用
                    }
                }
                
                if (lv_obj_is_valid(guider_ui.screen_3_btn_med2)) {
                    if (g_ui_data[uid].med_status[1] == 1) {
                        if (lv_obj_is_valid(guider_ui.screen_3_btn_med2_label)) 
                            lv_label_set_text(guider_ui.screen_3_btn_med2_label, "已服");
                        lv_obj_add_state(guider_ui.screen_3_btn_med2, LV_STATE_DISABLED);
                    } else {
                        if (lv_obj_is_valid(guider_ui.screen_3_btn_med2_label)) 
                            lv_label_set_text(guider_ui.screen_3_btn_med2_label, "吃药");
                        lv_obj_clear_state(guider_ui.screen_3_btn_med2, LV_STATE_DISABLED);
                    }
                }

                if (lv_obj_is_valid(guider_ui.screen_3_btn_med3)) {
                    if (g_ui_data[uid].med_status[2] == 1) {
                        if (lv_obj_is_valid(guider_ui.screen_3_btn_med3_label)) 
                            lv_label_set_text(guider_ui.screen_3_btn_med3_label, "已服");
                        lv_obj_add_state(guider_ui.screen_3_btn_med3, LV_STATE_DISABLED);
                    } else {
                        if (lv_obj_is_valid(guider_ui.screen_3_btn_med3_label)) 
                            lv_label_set_text(guider_ui.screen_3_btn_med3_label, "吃药");
                        lv_obj_clear_state(guider_ui.screen_3_btn_med3, LV_STATE_DISABLED);
                    }
                }
            
            }
            else if(g_ui_data[uid].screen_3_update_step==false)
            {
                if (lv_obj_is_valid(guider_ui.screen_3_label_1)){
               lv_label_set_text(guider_ui.screen_3_label_1, "不在用药时间"); 
                }
            }
        }

           if(g_ui_data[uid].update_flags & UI_FLAG_USER_STEP) 
                {
            if (guider_ui.screen_label_user != NULL) 
            {
                if (uid == 0) {
               lv_label_set_text_fmt(guider_ui.screen_label_user, "User: %d", uid+1);
            } else if (uid == 1) {
                lv_label_set_text_fmt(guider_ui.screen_label_user, "User: %d", uid+1);
            } else if (uid == 2) {
                lv_label_set_text_fmt(guider_ui.screen_label_user, "User: %d", uid+1);
            } 
            }
        }
            //更新城市
            if(g_ui_data[uid].update_flags & UI_FLAG_CITY)
            {
                if (guider_ui.screen_label_city != NULL) 
                {
                    lv_label_set_text_fmt(guider_ui.screen_label_city, "City: %s", g_ui_data[uid].city_name);
                }
            }
            //更新天气
            if(g_ui_data[uid].update_flags & UI_FLAG_WEATHER)
            {
                if (guider_ui.screen_label_weather_icon != NULL) 
                {
                    lv_label_set_text_fmt(guider_ui.screen_label_weather_icon, "weather:%s", g_ui_data[uid].weather);
                }
            }
            //更新温度
            if(g_ui_data[uid].update_flags & UI_FLAG_TEMPERATURE)
            {
                if (guider_ui.screen_label_temperature != NULL) 
                {
                    lv_label_set_text_fmt(guider_ui.screen_label_temperature, "temperature:%d°C", g_ui_data[uid].weather_temp);
                }
            }

             // 刷新完成后，清除更新标志
            g_ui_data[uid].update_flags = 0; 
            xSemaphoreGive(xGuiMutex);
        }
        }

        /*检查是否到吃药时间刷新*/
        /*每1秒检查一次*/
       // 每 1秒无阻塞触发一次药盒检查
        if (xTaskGetTickCount() - last_check_mechine_time >= pdMS_TO_TICKS(1000)) {
            check_medication_time();
            last_check_mechine_time = xTaskGetTickCount(); // 更新上一次检查的时间
        }
        //设置看门狗事件位，表示 sensor_lcd_task 正常运行
        xEventGroupSetBits(xWatchdogEventGroup, WDOG_BIT_SENSOR_LCD_TASK);
        vTaskDelay(pdMS_TO_TICKS(5)); // 每5ms更新一次 LCD
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
                    // APP_LOG("[App] Received CMD: Send SMS Alert\n");
                    // if (!a7670c_ready) {
                    //     printf("[App] A7670C not ready, cannot send SMS!\n");
                    //     break;
                    // }
                    // A7670C_WakeUp();
                    // APP_LOG("--- Alert triggered: Sending SMS ---\r\n");
                    // A7670C_SendSMS_Auto("18135183446", "请按时服药！");
                    // vTaskDelay(pdMS_TO_TICKS(1000));
                    // A7670C_EnterSleep();
                    // APP_LOG("--- SMS sent, module sleeping ---\r\n");
                    break;

                default:
                    break;
            }
        }
     
         //设置看门狗事件位，表示 bsp_sensor_task 正常运行
        xEventGroupSetBits(xWatchdogEventGroup, WDOG_BIT_SENSOR_TASK);
        vTaskDelay(pdMS_TO_TICKS(5)); // 每5ms检查一次系统指令队列
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
                    // 修改 RTC 时间
                    BSP_RTC_ModifyTime(2000 + year, month, day, hour, minute, second);
                    
                    last_sync_tick = current_tick;
                    is_first_sync = 0;

                    // 立即更新 LCD 显示的时间
                    BSP_RTC_GetDateTime(&g_ui_data[g_current_active_user_id].time);
                    g_ui_data[g_current_active_user_id].update_flags |= UI_FLAG_TIME;
                    
                    printf("[Time Sync] RTC & LCD display calibrated successfully!\n");
                }
            }

            
    /*位置同步*/
if (rx_packet.sensor_id == CMD_LOCATION_SYNC && rx_packet.data_len >= 2)
{
    char city_name[32] = {0};  // 临时缓冲区更大一些
    
    // 限制复制长度 (这里拿到的还是中文，比如 "太原")
    size_t copy_len = (rx_packet.data_len < sizeof(city_name) - 1) ? 
                      rx_packet.data_len : sizeof(city_name) - 1;
    
    memcpy(city_name, rx_packet.payload, copy_len);
    city_name[copy_len] = '\0';
    
    // ================= 新增转换逻辑 =================
    // 将拿到的中文城市名丢进函数，获取英文拼音
    const char* english_city = convert_city_to_english(city_name);
    // ================================================
    
    // 安全复制到全局数据 (此时复制进去的是 "Taiyuan")
    strncpy(g_ui_data[g_current_active_user_id].city_name, english_city, 
            sizeof(g_ui_data[g_current_active_user_id].city_name) - 1);
    g_ui_data[g_current_active_user_id].city_name[sizeof(g_ui_data[g_current_active_user_id].city_name) - 1] = '\0';
    
    APP_LOG("[Location Sync] City updated to: %s\n", 
    g_ui_data[g_current_active_user_id].city_name);
    
    g_ui_data[g_current_active_user_id].update_flags |= UI_FLAG_CITY;
}

            /*天气同步*/
if (rx_packet.sensor_id == CMD_WEATHER_SYNC && rx_packet.data_len >= 2)
{
    int8_t weather_code = (int8_t)rx_packet.payload[0]; // 天气代码
    int8_t temperature = (int8_t)rx_packet.payload[1];   // 温度
    
    // 心知天气代码映射（0-37常见值）
    switch(weather_code) {
        case 0:  strcpy(g_ui_data[g_current_active_user_id].weather, "Sunny"); break;      // 晴
        case 1:  strcpy(g_ui_data[g_current_active_user_id].weather, "Clear"); break;      // 晴（夜间）
        case 2:  strcpy(g_ui_data[g_current_active_user_id].weather, "Fair"); break;       // 少云
        case 3:  strcpy(g_ui_data[g_current_active_user_id].weather, "Cloudy"); break;     // 多云
        case 4:  strcpy(g_ui_data[g_current_active_user_id].weather, "Overcast"); break;   // 阴天
        
        case 5:  strcpy(g_ui_data[g_current_active_user_id].weather, "Fog"); break;        // 雾
        case 6:  strcpy(g_ui_data[g_current_active_user_id].weather, "Rain"); break;       // 雨
        
        case 7:  strcpy(g_ui_data[g_current_active_user_id].weather, "Snow"); break;       // 雪
        case 8:  strcpy(g_ui_data[g_current_active_user_id].weather, "Snow"); break;       // 雪
        case 9:  strcpy(g_ui_data[g_current_active_user_id].weather, "Snow"); break;       // 雪
        
        case 10: strcpy(g_ui_data[g_current_active_user_id].weather, "Rain"); break;       // 雨
        case 11: strcpy(g_ui_data[g_current_active_user_id].weather, "Rain"); break;       // 雨
        case 12: strcpy(g_ui_data[g_current_active_user_id].weather, "Rain"); break;       // 雨
        case 13: strcpy(g_ui_data[g_current_active_user_id].weather, "Shower"); break;     // 阵雨
        case 14: strcpy(g_ui_data[g_current_active_user_id].weather, "Shower"); break;     // 阵雨
        
        case 15: strcpy(g_ui_data[g_current_active_user_id].weather, "Thunder"); break;    // 雷雨
        case 16: strcpy(g_ui_data[g_current_active_user_id].weather, "Thunder"); break;    // 雷雨
        
        case 17: strcpy(g_ui_data[g_current_active_user_id].weather, "Snow"); break;       // 雪
        case 18: strcpy(g_ui_data[g_current_active_user_id].weather, "Snow"); break;       // 雪
        
        case 19: strcpy(g_ui_data[g_current_active_user_id].weather, "Rain"); break;       // 冻雨
        
        default: strcpy(g_ui_data[g_current_active_user_id].weather, "Unknown"); break;
    }
    
    g_ui_data[g_current_active_user_id].weather_temp = temperature;
    g_ui_data[g_current_active_user_id].update_flags |= UI_FLAG_WEATHER;
    
    APP_LOG("[Weather Sync] Weather: %s (code=%d), Temp: %d°C\n", 
            g_ui_data[g_current_active_user_id].weather, weather_code, temperature);
}

            /*wifi状态更新*/
            if (rx_packet.sensor_id == CMD_WIFI_STATUS_UPDATE)
            {
                
                //APP_LOG("[WiFi] Status update received: 0x%02X\n", rx_packet.payload[0]);
                if(rx_packet.payload[0] == 0x32) {
                    // 接收到心跳包，说明模块在线，更新 WiFi 状态和最后收到状态的时间戳
                    last_wifi_tick = xTaskGetTickCount();
                    g_ui_data[g_current_active_user_id].wifi_status = 0; // 0 表示 WiFi 已连接
                } else {
                    g_ui_data[g_current_active_user_id].wifi_status = 1; // 1 表示 WiFi 断开
                }
                g_ui_data[g_current_active_user_id].update_flags |= UI_FLAG_WIFI; // 触发 UI 刷新
            }
        }
       // ==========================================
        // 3. 独立的心跳超时检测逻辑 (放在接收 if 块的外面)
        // ==========================================
        /* 串口 20s 内没有接收到任何数据时，把 wifi_status 设置为 'F' */
        if(g_ui_data[g_current_active_user_id].wifi_status == 0) 
        {
            if ((xTaskGetTickCount() - last_wifi_tick) >= pdMS_TO_TICKS(20000)) 
            {
                APP_LOG("[WiFi] No status update received for 20s, setting status to 'off'\n");
                g_ui_data[g_current_active_user_id].wifi_status = 1;
                g_ui_data[g_current_active_user_id].update_flags |= UI_FLAG_WIFI; // 触发 UI 刷新
            }
        }
        //设置看门狗事件位，表示 usart_task 正常运行
        xEventGroupSetBits(xWatchdogEventGroup, WDOG_BIT_USART_TASK);
        vTaskDelay(pdMS_TO_TICKS(5));
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
        if(WDOG_BIT_ALL_TASKS == (xEventGroupGetBits(xWatchdogEventGroup) & WDOG_BIT_ALL_TASKS)) {

             IWDG_ReloadCounter(); // 喂独立看门狗
            xEventGroupClearBits(xWatchdogEventGroup, WDOG_BIT_ALL_TASKS); // 清除所有任务位，准备下一轮监测
        }
            vTaskDelay(pdMS_TO_TICKS(100));
    }

}

/*规定时间为前后10分钟*/ // 注：代码里写的是 +/- 1分钟
void check_medication_time(void) {
    RTC_TimeTypeDef current_time;
    BSP_RTC_GetDateTime(&current_time);
    
    int16_t curr_total_mins = current_time.hour * 60 + current_time.min;
    
    uint8_t is_time_to_eat = 0; 
    int8_t active_med_idx = -1; // 记录当前是哪一顿药触发了时间
    
    //APP_LOG("[App] Checking medication time... Current time: %02d:%02d\n", current_time.hour, current_time.min);
    
    // 遍历检查设定的3个吃药时间
    for (int i = 0; i < MAX_SCHE; i++) {
        int16_t target_total_mins = my_meds[i].hour * 60 + my_meds[i].min;
        int16_t diff_mins = curr_total_mins - target_total_mins;
        
        // 处理跨天的情况
        if (diff_mins > 12 * 60) diff_mins -= 24 * 60;
        else if (diff_mins < -12 * 60) diff_mins += 24 * 60;
        
        //APP_LOG("[App] Checking med #%d: target time %02d:%02d, diff_mins = %d\n", i+1, my_meds[i].hour, my_meds[i].min, diff_m
        if (diff_mins >= -10 &&diff_mins <= 0) { // 当前时间在吃药时间的前10分钟到吃药时间之间
            is_time_to_eat = 1; 
            active_med_idx = i; // 把当前触发的这顿药的编号记下来
            break; 
        }
    }
    
    if (is_time_to_eat) {
        // 使用静态变量记录上一次重置状态的【日期】和【药品索引】
        static int8_t last_reset_med_idx = -1;
        static uint8_t last_reset_day = 0xFF;

        // 核心判断：如果今天还没有为这顿药重置过状态，才执行重置、存 Flash、播报语音
        if (last_reset_day != current_time.day || last_reset_med_idx != active_med_idx) {
            
            g_ui_data[g_current_active_user_id].meds_completed[0] = 0;
            g_ui_data[g_current_active_user_id].meds_completed[1] = 0;
            g_ui_data[g_current_active_user_id].meds_completed[2] = 0;

            g_ui_data[g_current_active_user_id].med_status[0] = 0;
            g_ui_data[g_current_active_user_id].med_status[1] = 0;    
            g_ui_data[g_current_active_user_id].med_status[2] = 0;    
            
            /*并保存到Flash*/
            SystemData_Save_To_Flash_ByUser(g_current_active_user_id);
            
            // 更新记录，打上"已处理"的标签
            last_reset_med_idx = active_med_idx;
            last_reset_day = current_time.day;

            g_ui_data[g_current_active_user_id].screen_3_update_step = true;  
            
            // // 只有刚刚进入吃药时间的那一刻，才自动发数据帧让天问播报
            // Tianwen_Packet_t tw_pkt = {0};
            // tw_pkt.id = CMD_TX_TIME_TO_EAT;
            // xQueueSend(sendtianwenQueue, &tw_pkt, 0);
            // APP_LOG("[Medication Time] It's time to take medicine #%d! Resetting status and triggering voice reminder.\n", active_med_idx + 1);
        }
       
    } else {
        g_ui_data[g_current_active_user_id].screen_3_update_step = false;  
    }
}


void Send_Missed_Medication_Record(uint8_t user_id, uint8_t med_idx, 
                                   uint8_t year, uint8_t month, uint8_t day, 
                                   uint8_t hour, uint8_t min, uint8_t sec)
{
    Packet_t pkt;
    
    // 1. 填充帧头信息
    pkt.data_len = 7;                            // 负载长度：1字节顿数 + 6字节时间戳
    pkt.sensor_id = CMD_UPLOAD_MISSED_MED;       // 指令/命令ID：0x20 (漏服)
    pkt.user_id = user_id+1;                       // 当前数据归属的用户ID
    
    // 2. 组装 Payload (数据负载)
    pkt.payload[0] = med_idx;                    // 漏服的顿数
    pkt.payload[1] = year;                       // 年
    pkt.payload[2] = month;                      // 月
    pkt.payload[3] = day;                        // 日
    pkt.payload[4] = hour;                       // 时
    pkt.payload[5] = min;                        // 分
    pkt.payload[6] = sec;                        // 秒
    
    // 3. 发送给 ESP32
    // 方式A: 直接调用底层串口/DMA发送 (会在此处阻塞等待发送完成)
    USART_WIFI_ESP_Send(&pkt); 
    
    // 方式B: 推荐做法！压入 RingBuffer，由专用的 usart_task 异步发送，不阻塞当前任务
    // RingBuffer_push(&g_ring_buffer, &pkt);
}

// 3. 核心查找函数
const char* convert_city_to_english(const char* chinese_name) {
    // 安全校验：防止传入空指针导致单片机死机崩溃
    if (chinese_name == NULL) {
        return "Unknown";
    }

    // 计算数组中共有多少个城市
    int map_size = sizeof(city_map) / sizeof(city_map[0]);

    // 遍历字典查找匹配项
    for (int i = 0; i < map_size; i++) {
        // 使用 strstr 兼容网络返回的 "北京市" 或 "北京"
        if (strstr(chinese_name, city_map[i].zh) != NULL) {
            return city_map[i].en;
        }
    }

    // 如果遍历完都没有找到匹配的，返回默认兜底英文字符串
    return "Unknown City"; 
}