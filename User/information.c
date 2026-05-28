#include "information.h"
#include "spi_w25q.h"
#include <string.h>
#include <stdio.h>
#include "information.h"
#include "../bsp/bsp_a7670c/bsp_a7670c.h"

#define DATA_SECTOR_ADDR  0x7F8000
#define SECTOR_SIZE       4096
// 移除了原来的 DEFAULT_ALARM 固定数组，统一使用 my_meds 管理
extern  UI_DisplayData_t g_ui_data[MAX_USER]; // 全局 UI 数据结构实例
// 记录当前屏幕正在操作哪个用户
uint8_t g_current_active_user_id = 2;/*可选0，1，2...*/

static uint32_t current_flash_offset[MAX_USER] = {0};
AlarmSche my_meds[3] = {
    {0, 2, 1},
    {0, 5, 1},
    {0, 8, 1}
};
uint16_t current_time_virvual[6]={2026,5,28,0,0,50}; // 0-年高8位，1-年低8位，2-月，3-日，4-时，5-分

void SystemData_Init_Load(void) 
{
    UI_DisplayData_t temp_data;
    
    // 【关键控制】开机结束后，默认当前屏幕操作 0 号用户
    g_current_active_user_id = 0; 

    // 循环遍历加载所有用户的数据，实现真正的数据隔离初始化
    for (uint8_t uid = 0; uid < MAX_USER; uid++)
    {
        bool found = false;
        current_flash_offset[uid] = 0;

        // 1. 修复：使用 uid 隔离初始化对应用户的内存空间
        memset(&g_ui_data[uid], 0, sizeof(UI_DisplayData_t));
        g_ui_data[uid].magic_num = 0x55AA;
        g_ui_data[uid].wifi_status = 'F'; // 开机默认显示断开 'F'

        // 计算该用户在 W25Q 中的独立扇区基地址
        uint32_t user_base_addr = DATA_SECTOR_ADDR + (uid * SECTOR_SIZE);

        // 2. 步进扫描该用户的整个 4KB 扇区，寻找最后一次滚动写入的有效数据
        for (uint32_t offset = 0; offset + sizeof(UI_DisplayData_t) <= SECTOR_SIZE; offset += sizeof(UI_DisplayData_t)) 
        {
            W25Q_ReadData(user_base_addr + offset, (uint8_t *)&temp_data, sizeof(UI_DisplayData_t));
            
            /* 检测魔术字，如果遇到 0xFFFF 说明后面全是空白片，直接退出扫描 */
            if (temp_data.magic_num == 0xFFFF) {
                break;
            }
            
            /* 如果匹配到有效魔术字，说明找到了一个历史写入点，暂存并继续往后找最新的 */
            if (temp_data.magic_num == 0x55AA) {
                // 修复：拷贝给当前遍历的 uid，而不是固定的 g_current_active_user_id
                memcpy(&g_ui_data[uid], &temp_data, sizeof(UI_DisplayData_t));
                
                // WiFi 状态是动态的，不从 Flash 恢复
                g_ui_data[uid].wifi_status = 'F'; 
                
                // 记住该用户当前的滚动偏移量
                current_flash_offset[uid] = offset;
                found = true;
            }
        }

        /* 兼容老数据修正：漏服记录最多只看 4 条 */
        if (g_ui_data[uid].record_count > 4) {
            g_ui_data[uid].record_count = 4;
        }

        // 3. 如果在整个扇区里都没找到有效数据 (说明该用户的扇区是首次使用的空片)
        if (!found) {
            memset(&g_ui_data[uid], 0, sizeof(UI_DisplayData_t));
            g_ui_data[uid].magic_num = 0x55AA;
            g_ui_data[uid].wifi_status = 'F';
            
            // 只有空片时，才引入系统默认吃药时间计划和默认数量
            for (int i = 0; i < MAX_SCHE; i++) {
                g_ui_data[uid].meds_schedule[i].hour = my_meds[i].hour;
                g_ui_data[uid].meds_schedule[i].min  = my_meds[i].min;
                g_ui_data[uid].meds_schedule[i].pill_count = 1;
            }

            // 擦除该用户独立的 4KB 扇区并写入初始数据
            W25Q_SectorErase(user_base_addr);
            W25Q_WriteBuffer(user_base_addr, (uint8_t *)&g_ui_data[uid], sizeof(UI_DisplayData_t));
            current_flash_offset[uid] = 0;
        }
        
        // 4. 【重要逻辑调整】：不再用单组 my_meds 强行覆盖已经读出来的、属于不同用户的独立时间！
        // 如果需要让任务层知道当前选中的 0 号用户的时间，可以在循环外单独对齐：

        // 开机强制刷新该用户对应的整个 UI 标志位
        g_ui_data[uid].update_flags = 0xFFFFFFFF; 
    }

    // 5. 循环结束后，为后台业务逻辑变量 my_meds 动态赋予当前默认活跃用户（0号用户）的时间
    memcpy(my_meds, g_ui_data[g_current_active_user_id].meds_schedule, sizeof(my_meds));
}

/**
 * @brief  保存指定用户的数据到 Flash 独立滚动扇区
 * @param  user_idx: 用户索引 (0 ~ MAX_USER-1)
 */
void SystemData_Save_To_Flash_ByUser(uint8_t user_idx) 
{
    if (user_idx >= MAX_USER) return; // 防御性判断

    // 1. 计算该用户的独立扇区首地址 (User 0: 0x7F8000, User 1: 0x7F9000...)
    uint32_t user_base_addr = DATA_SECTOR_ADDR + (user_idx * SECTOR_SIZE);

    // 2. 计算下一次写入的偏移量
    uint32_t next_offset = current_flash_offset[user_idx] + sizeof(UI_DisplayData_t);

    // 3. 检查当前扇区剩余空间是否还够下一次写入
    if (next_offset + sizeof(UI_DisplayData_t) > SECTOR_SIZE) {
        // 空间不够了，擦除该用户独立的 4KB 扇区
        W25Q_SectorErase(user_base_addr);
        current_flash_offset[user_idx] = 0;           
    } else {
        // 空间足够，累加偏移量
        current_flash_offset[user_idx] = next_offset; 
    }

    // 4. 动态写入该用户的数据到其独立扇区的滚动位置
    uint32_t final_write_addr = user_base_addr + current_flash_offset[user_idx];
    W25Q_WriteBuffer(final_write_addr, (uint8_t *)&g_ui_data[user_idx], sizeof(UI_DisplayData_t));
}

// ============== 下面是所有数据的追加函数 (都会触发 Flash 保存) ==============

void Add_Missed_Record(uint32_t new_id, const char *new_date, const char *new_time) {
    /* 最多保存 4 条漏服记录，下标 0 总是最新 */
    #define MAX_MISSED_STORE 4

    MissedDoseRecord new_rec = {0};
    new_rec.id = new_id;
    strncpy(new_rec.date, new_date, sizeof(new_rec.date) - 1);
    strncpy(new_rec.time, new_time, sizeof(new_rec.time) - 1);

    if (g_ui_data[g_current_active_user_id].record_count < MAX_MISSED_STORE) {
        /* 不满 4 条：整体后移，新记录插在最前面 */
        for (int i = g_ui_data[g_current_active_user_id].record_count; i > 0; i--) {
            g_ui_data[g_current_active_user_id].missed_records[i] = g_ui_data[g_current_active_user_id].missed_records[i - 1];
        }
        g_ui_data[g_current_active_user_id].missed_records[0] = new_rec;
        g_ui_data[g_current_active_user_id].record_count++;
    } else {
        /* 已满 4 条：丢弃最后一条（最早的），整体后移，新记录插最前面 */
        for (int i = MAX_MISSED_STORE - 1; i > 0; i--) {
            g_ui_data[g_current_active_user_id].missed_records[i] = g_ui_data[g_current_active_user_id].missed_records[i - 1];
        }
        g_ui_data[g_current_active_user_id].missed_records[0] = new_rec;
    }
    g_ui_data[g_current_active_user_id].update_flags |= UI_FLAG_RECORDS;
    SystemData_Save_To_Flash_ByUser(g_current_active_user_id);
}

void Add_History_HR_SpO2(uint16_t hr, uint16_t spo2) {
    History_HR_SpO2_t new_rec = {0};
    snprintf(new_rec.time, sizeof(new_rec.time), "%02d-%02d %02d:%02d", 
             g_ui_data[g_current_active_user_id].time.month, g_ui_data[g_current_active_user_id].time.day, g_ui_data[g_current_active_user_id].time.hour, g_ui_data[g_current_active_user_id].time.min);
    new_rec.hr = hr;
    new_rec.spo2 = spo2;

    if (g_ui_data[g_current_active_user_id].hr_count < MAX_HISTORY) {
        g_ui_data[g_current_active_user_id].hr_history[g_ui_data[g_current_active_user_id].hr_count++] = new_rec;
    } else {
        for (int i = 0; i < MAX_HISTORY - 1; i++) g_ui_data[g_current_active_user_id].hr_history[i] = g_ui_data[g_current_active_user_id].hr_history[i + 1];
        g_ui_data[g_current_active_user_id].hr_history[MAX_HISTORY - 1] = new_rec;
    }
    g_ui_data[g_current_active_user_id].update_flags |= UI_FLAG_HR_SPO2;
    SystemData_Save_To_Flash_ByUser(g_current_active_user_id);
}

void Add_History_BodyTemp(uint16_t temp) {
    History_BodyTemp_t new_rec = {0};
    snprintf(new_rec.time, sizeof(new_rec.time), "%02d-%02d %02d:%02d", 
             g_ui_data[g_current_active_user_id].time.month, g_ui_data[g_current_active_user_id].time.day, g_ui_data[g_current_active_user_id].time.hour, g_ui_data[g_current_active_user_id].time.min);
    new_rec.body_temp = temp;

    if (g_ui_data[g_current_active_user_id].bt_count < MAX_HISTORY) {
        g_ui_data[g_current_active_user_id].bt_history[g_ui_data[g_current_active_user_id].bt_count++] = new_rec;
    } else {
        for (int i = 0; i < MAX_HISTORY - 1; i++) g_ui_data[g_current_active_user_id].bt_history[i] = g_ui_data[g_current_active_user_id].bt_history[i + 1];
        g_ui_data[g_current_active_user_id].bt_history[MAX_HISTORY - 1] = new_rec;
    }
    g_ui_data[g_current_active_user_id].update_flags |= UI_FLAG_BODY_TEMP;
    SystemData_Save_To_Flash_ByUser(g_current_active_user_id);
}

void Add_History_EnvTH(uint16_t temp, uint16_t humi) {
    History_EnvTempHumi_t new_rec = {0};
    snprintf(new_rec.time, sizeof(new_rec.time), "%02d-%02d %02d:%02d", 
             g_ui_data[g_current_active_user_id].time.month, g_ui_data[g_current_active_user_id].time.day, g_ui_data[g_current_active_user_id].time.hour, g_ui_data[g_current_active_user_id].time.min);
    new_rec.env_temp = temp;
    new_rec.env_humi = humi;

    if (g_ui_data[g_current_active_user_id].env_count < MAX_HISTORY) {
        g_ui_data[g_current_active_user_id].env_history[g_ui_data[g_current_active_user_id].env_count++] = new_rec;
    } else {
        for (int i = 0; i < MAX_HISTORY - 1; i++) g_ui_data[g_current_active_user_id].env_history[i] = g_ui_data[g_current_active_user_id].env_history[i + 1];
        g_ui_data[g_current_active_user_id].env_history[MAX_HISTORY - 1] = new_rec;
    }
    g_ui_data[g_current_active_user_id].update_flags |= UI_FLAG_ENV_TH;
    SystemData_Save_To_Flash_ByUser(g_current_active_user_id);
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
        if (g_ui_data[g_current_active_user_id].meds_completed[idx]==true) {
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
    SystemData_Save_To_Flash_ByUser(g_current_active_user_id);
}
