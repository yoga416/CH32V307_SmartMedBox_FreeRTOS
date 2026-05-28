#ifndef __INFORMATION_H
#define __INFORMATION_H

#include <stdint.h>
#include <stdbool.h>
#include "ch32v30x.h"
#include "bsp_rtc.h"
#include "lvgl.h"
#define MAX_SCHE 3
#define MAX_HISTORY 10      // 历史记录最大保存次数
#define MAX_USER  3        // 最大用户数量
// ==========================================
// 刷新标志位 (Dirty Flags)
// ==========================================
#define UI_FLAG_HR_SPO2       (1 << 0)
#define UI_FLAG_BODY_TEMP     (1 << 1)
#define UI_FLAG_ENV_TH        (1 << 2)
#define UI_FLAG_TIME          (1 << 3)
#define UI_FLAG_WIFI          (1 << 4)
#define UI_FLAG_RECORDS       (1 << 5)
#define UI_FLAG_SCHEDULE      (1 << 6)
#define UI_FLAG_MED_STATUS    (1 << 7) // 吃药状态更新
#define UI_FLAG_USER_STEP     (1 << 8) // 吃药界面分步刷新控制
/*时间开始*/
extern uint16_t current_time_virvual[6]; // 0-年高8位，1-年低8位，2-月，3-日，4-时，5-分

// ==========================================
// 数据类型定义
// ==========================================
typedef struct {
    uint32_t id;
    char date[12];
    char time[8];
    uint8_t reserved[8];
} MissedDoseRecord;

typedef struct {
    uint8_t hour;
    uint8_t min;
    uint8_t pill_count;  // 这个有界面的选项，但目前界面上没有实现修改功能，暂时写死为1
} AlarmSche;

// 【新增】心率血氧历史记录 (16 字节)
typedef struct {
    char time[12];       // 时间戳 "05-20 10:30"
    uint16_t hr;         // 心率
    uint16_t spo2;       // 血氧
} History_HR_SpO2_t;

// 【新增】体温历史记录 (16 字节)
typedef struct {
    char time[12];       // 时间戳 "05-20 10:30"
    uint16_t body_temp;  // 体温
    uint8_t reserved[2]; // 内存对齐
} History_BodyTemp_t;

// 【新增】环境温湿度历史记录 (16 字节)
typedef struct {
    char time[12];       // 时间戳 "05-20 10:30"
    uint16_t env_temp;   // 环境温度
    uint16_t env_humi;   // 环境湿度
} History_EnvTempHumi_t;


// ==========================================
// 全局核心系统数据结构体
// ==========================================
typedef struct {
    uint16_t magic_num;                      // 魔术字 0x55AA
    char wifi_status;                        // WIFI 状态
    uint8_t reserved_align;                  // 对齐

    // 1. 漏服记录
    uint16_t record_count;                   
    MissedDoseRecord missed_records[MAX_HISTORY]; 

    // 2. 心率血氧追溯 (保存10次)
    uint16_t hr_count;
    History_HR_SpO2_t hr_history[MAX_HISTORY];

    // 3. 体温追溯 (保存10次)
    uint16_t bt_count;
    History_BodyTemp_t bt_history[MAX_HISTORY];

    // 4. 环境温湿度追溯 (保存10次)
    uint16_t env_count;
    History_EnvTempHumi_t env_history[MAX_HISTORY];

    // 5. 用药闹钟配置
    AlarmSche meds_schedule[MAX_SCHE];       

    /*药品状态*/
    uint8_t med_status[3];  // 0:未服用, 1:已服用

    /*一个时间段内是否吃药完成*/
    uint8_t meds_completed[3]; // 0:未完成, 1:完成
    // 7. 实时状态（不存 Flash 的运行状态）
    RTC_TimeTypeDef time;

    uint32_t update_flags; 

    bool screen_3_update_step; // 0/1，控制吃药界面分步刷新
                      
} UI_DisplayData_t;

extern UI_DisplayData_t g_ui_data[MAX_USER];
extern uint8_t g_current_active_user_id; // 记录当前操作的用户 ID
// ==========================================
// 接口函数声明
// =========================================

/*UI_DisplayData_t的初始化加载和保存函数*/

/*初始化添加函数*/
void SystemData_Init_Load(void);

/*保存数据到Flash*/
void SystemData_Save_To_Flash_ByUser(uint8_t user_idx);

/*追加漏服记录*/
void Add_Missed_Record(uint32_t new_id, const char *new_date, const char *new_time);

/*追加心率血氧记录*/
void Add_History_HR_SpO2(uint16_t hr, uint16_t spo2);

/*追加体温记录*/
void Add_History_BodyTemp(uint16_t temp);

/*追加环境温湿度记录*/
void Add_History_EnvTH(uint16_t temp, uint16_t humi);



#endif // __INFORMATION_H