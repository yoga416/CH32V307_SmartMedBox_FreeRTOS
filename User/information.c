#include "information.h"
#include "spi_w25q.h"
#include <string.h>
#include <stdio.h>
#include "information.h"
#define DATA_SECTOR_ADDR  0x7F8000
#define SECTOR_SIZE       4096
#include "../bsp/bsp_a7670c/bsp_a7670c.h"
// 移除了原来的 DEFAULT_ALARM 固定数组，统一使用 my_meds 管理
extern UI_DisplayData_t g_ui_data;
static uint32_t current_flash_offset = 0;

AlarmSche my_meds[3] = {
    {0, 2, 1},
    {0, 5, 1},
    {0, 8, 1}
};
uint16_t current_time_virvual[6]={2026,5,28,0,0,50}; // 0-年高8位，1-年低8位，2-月，3-日，4-时，5-分
void SystemData_Init_Load(void) 
{
    UI_DisplayData_t temp_data;
    bool found = false;
    current_flash_offset = 0;

    // 先初始化为默认值
    memset(&g_ui_data, 0, sizeof(UI_DisplayData_t));
    g_ui_data.magic_num = 0x55AA;
    g_ui_data.wifi_status = 'F';

    for (uint32_t offset = 0; offset + sizeof(UI_DisplayData_t) <= SECTOR_SIZE; offset += sizeof(UI_DisplayData_t)) 
    {
        W25Q_ReadData(DATA_SECTOR_ADDR + offset, (uint8_t *)&temp_data, sizeof(UI_DisplayData_t));
        
        /*检测w25q是否写入过数据*/
        if (temp_data.magic_num == 0xFFFF) break;
        
        if (temp_data.magic_num == 0x55AA) {
            memcpy(&g_ui_data, &temp_data, sizeof(UI_DisplayData_t));
            
        // WiFi 状态是动态的，不从 Flash 恢复，开机默认显示断开 'F'
        g_ui_data.wifi_status = 'F'; 
            
        // 3. 【关键修复】既然时间由代码管理，强制用 my_meds 覆盖 Flash 里的老时间！
        for (int i = 0; i < MAX_SCHE; i++) { // 假设 MAX_SCHE 是 3
        g_ui_data.meds_schedule[i].hour = my_meds[i].hour;
        g_ui_data.meds_schedule[i].min  = my_meds[i].min;
        // pill_count 已经在 memcpy 时恢复了老数据，所以这里不需要再额外赋值
        }

        // /*接入云端后的代码*/
        // for (int i = 0; i < MAX_SCHE; i++) { // 假设 MAX_SCHE 是 3
        // my_meds[i].hour= temp_data.meds_schedule[i].hour;
        // my_meds[i].min= temp_data.meds_schedule[i].min;
        // my_meds[i].pill_count = temp_data.meds_schedule[i].pill_count;
        // // pill_count 已经在 memcpy 时恢复了老数据，所以这里不需要再额外赋值
        // }
            current_flash_offset = offset;
            found = true;
        }
    }

    /*原始数据*/
    /* 兼容旧数据：漏服记录最多只看 4 条 */
    if (g_ui_data.record_count > 4) g_ui_data.record_count = 4;

    if (!found) {
        memset(&g_ui_data, 0, sizeof(UI_DisplayData_t));
        g_ui_data.magic_num = 0x55AA;
        g_ui_data.wifi_status = 'F'; // 默认无 WiFi连接
        
        g_ui_data.meds_schedule[0].hour =my_meds[0].hour;
        g_ui_data.meds_schedule[0].min  = my_meds[0].min;
        g_ui_data.meds_schedule[0].pill_count = 1;

        g_ui_data.meds_schedule[1].hour = my_meds[1].hour;
        g_ui_data.meds_schedule[1].min  = my_meds[1].min;
        g_ui_data.meds_schedule[1].pill_count = 1;

        g_ui_data.meds_schedule[2].hour = my_meds[2].hour;
        g_ui_data.meds_schedule[2].min  = my_meds[2].min;
        g_ui_data.meds_schedule[2].pill_count = 1;

        W25Q_SectorErase(DATA_SECTOR_ADDR);
        W25Q_WriteBuffer(DATA_SECTOR_ADDR, (uint8_t *)&g_ui_data, sizeof(UI_DisplayData_t));
        current_flash_offset = 0;
    }
    
    // 开机强制刷新整个 UI
    g_ui_data.meds_schedule[0].hour = my_meds[0].hour;
    g_ui_data.meds_schedule[0].min = my_meds[0].min;
    g_ui_data.meds_schedule[1].hour = my_meds[1].hour;
    g_ui_data.meds_schedule[1].min = my_meds[1].min;
    g_ui_data.meds_schedule[2].hour = my_meds[2].hour; 
    g_ui_data.meds_schedule[2].min = my_meds[2].min;

    // 同步到实时变量
    //memcpy(my_meds, g_ui_data.meds_schedule, sizeof(my_meds));

    // 开机强制刷新整个 UI
    g_ui_data.update_flags = 0xFFFFFFFF; 
}

void SystemData_Save_To_Flash(void) 
{
    uint32_t next_offset = current_flash_offset + sizeof(UI_DisplayData_t);

    if (next_offset + sizeof(UI_DisplayData_t) > SECTOR_SIZE) {
        W25Q_SectorErase(DATA_SECTOR_ADDR);
        current_flash_offset = 0;           
    } else {
        current_flash_offset = next_offset; 
    }

    W25Q_WriteBuffer(DATA_SECTOR_ADDR + current_flash_offset, (uint8_t *)&g_ui_data, sizeof(UI_DisplayData_t));
}

// ============== 下面是所有数据的追加函数 (都会触发 Flash 保存) ==============

void Add_Missed_Record(uint32_t new_id, const char *new_date, const char *new_time) {
    /* 最多保存 4 条漏服记录，下标 0 总是最新 */
    #define MAX_MISSED_STORE 4

    MissedDoseRecord new_rec = {0};
    new_rec.id = new_id;
    strncpy(new_rec.date, new_date, sizeof(new_rec.date) - 1);
    strncpy(new_rec.time, new_time, sizeof(new_rec.time) - 1);

    if (g_ui_data.record_count < MAX_MISSED_STORE) {
        /* 不满 4 条：整体后移，新记录插在最前面 */
        for (int i = g_ui_data.record_count; i > 0; i--) {
            g_ui_data.missed_records[i] = g_ui_data.missed_records[i - 1];
        }
        g_ui_data.missed_records[0] = new_rec;
        g_ui_data.record_count++;
    } else {
        /* 已满 4 条：丢弃最后一条（最早的），整体后移，新记录插最前面 */
        for (int i = MAX_MISSED_STORE - 1; i > 0; i--) {
            g_ui_data.missed_records[i] = g_ui_data.missed_records[i - 1];
        }
        g_ui_data.missed_records[0] = new_rec;
    }
    g_ui_data.update_flags |= UI_FLAG_RECORDS;
    SystemData_Save_To_Flash();
}

void Add_History_HR_SpO2(uint16_t hr, uint16_t spo2) {
    History_HR_SpO2_t new_rec = {0};
    snprintf(new_rec.time, sizeof(new_rec.time), "%02d-%02d %02d:%02d", 
             g_ui_data.time.month, g_ui_data.time.day, g_ui_data.time.hour, g_ui_data.time.min);
    new_rec.hr = hr;
    new_rec.spo2 = spo2;

    if (g_ui_data.hr_count < MAX_HISTORY) {
        g_ui_data.hr_history[g_ui_data.hr_count++] = new_rec;
    } else {
        for (int i = 0; i < MAX_HISTORY - 1; i++) g_ui_data.hr_history[i] = g_ui_data.hr_history[i + 1];
        g_ui_data.hr_history[MAX_HISTORY - 1] = new_rec;
    }
    g_ui_data.update_flags |= UI_FLAG_HR_SPO2;
    SystemData_Save_To_Flash();
}

void Add_History_BodyTemp(uint16_t temp) {
    History_BodyTemp_t new_rec = {0};
    snprintf(new_rec.time, sizeof(new_rec.time), "%02d-%02d %02d:%02d", 
             g_ui_data.time.month, g_ui_data.time.day, g_ui_data.time.hour, g_ui_data.time.min);
    new_rec.body_temp = temp;

    if (g_ui_data.bt_count < MAX_HISTORY) {
        g_ui_data.bt_history[g_ui_data.bt_count++] = new_rec;
    } else {
        for (int i = 0; i < MAX_HISTORY - 1; i++) g_ui_data.bt_history[i] = g_ui_data.bt_history[i + 1];
        g_ui_data.bt_history[MAX_HISTORY - 1] = new_rec;
    }
    g_ui_data.update_flags |= UI_FLAG_BODY_TEMP;
    SystemData_Save_To_Flash();
}

void Add_History_EnvTH(uint16_t temp, uint16_t humi) {
    History_EnvTempHumi_t new_rec = {0};
    snprintf(new_rec.time, sizeof(new_rec.time), "%02d-%02d %02d:%02d", 
             g_ui_data.time.month, g_ui_data.time.day, g_ui_data.time.hour, g_ui_data.time.min);
    new_rec.env_temp = temp;
    new_rec.env_humi = humi;

    if (g_ui_data.env_count < MAX_HISTORY) {
        g_ui_data.env_history[g_ui_data.env_count++] = new_rec;
    } else {
        for (int i = 0; i < MAX_HISTORY - 1; i++) g_ui_data.env_history[i] = g_ui_data.env_history[i + 1];
        g_ui_data.env_history[MAX_HISTORY - 1] = new_rec;
    }
    g_ui_data.update_flags |= UI_FLAG_ENV_TH;
    SystemData_Save_To_Flash();
}


/*检查是否漏服的函数，封装成一个独立函数，方便在多个地方调用（定时检查和用户按键检查）*/
void check_missed_doses(void){
   
    char date_str[10];
    char time_str[10];
    RTC_TimeTypeDef current_time;
    BSP_RTC_GetDateTime(&current_time);

    static uint8_t recorded_minute[3] = {0xFF, 0xFF, 0xFF};
    int idx = -1;

    if(current_time.hour == my_meds[0].hour && current_time.min == my_meds[0].min ) {
        idx = 0;
    } 
    else if(current_time.hour == my_meds[1].hour && current_time.min == my_meds[1].min) {
        idx = 1;
    } 
    else if(current_time.hour == my_meds[2].hour && current_time.min == my_meds[2].min) {
        idx = 2;
    }

    if (idx >= 0) {
        /* 优先检查：已服用 → 直接返回，不记录漏服 */
        if (g_ui_data.meds_completed[idx]==true) {
            return;
        }
        /* 其次：每分钟只记录一次 */
        if (recorded_minute[idx] == current_time.min) {
            return;
        }
        recorded_minute[idx] = current_time.min;

        /*发送短信漏服提醒*/
        char sms_text[64];
        snprintf(sms_text, sizeof(sms_text), "您有药物[%d]未按时服用，请及时服药！", idx + 1);
        A7670C_SendSMS_Auto("18135183446", sms_text);
        snprintf(date_str, sizeof(date_str), "%02d-%02d", current_time.month, current_time.day);
        snprintf(time_str, sizeof(time_str), "%02d:%02d", my_meds[idx].hour, my_meds[idx].min);
        Add_Missed_Record((uint32_t)idx, date_str, time_str);
    }
    }
