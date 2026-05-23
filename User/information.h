#ifndef __INFORMATION_H
#define __INFORMATION_H

#include "../bsp_rtc/bsp_rtc.h"
/*定义漏服记录的结构体*/
typedef struct {
    int id;
    char date[11]; // "YYYY/MM/DD"
    char time[6];  // "HH:MM"
} MissedDoseRecord;


// 定义更新标志位 (Dirty Flags) - 只有对应位为 1，才会刷新对应 UI
#define UI_FLAG_HR_SPO2       (1 << 0)
#define UI_FLAG_BODY_TEMP     (1 << 1)
#define UI_FLAG_ENV_TH        (1 << 2)
#define UI_FLAG_TIME          (1 << 3)
#define UI_FLAG_WIFI          (1 << 4)
#define UI_FLAG_RECORDS       (1 << 5)
#define UI_FLAG_SCHEDULE      (1 << 6)

typedef struct {
    // 1. 健康监测数据 (心率、血氧、体温)
    uint16_t heart_rate;
    uint16_t spo2;
    uint16_t body_temp;
    
    // 2. 环境监测数据
    uint16_t env_temp;
    uint16_t env_humi;
    
    // 3. 系统状态与时间
    RTC_TimeTypeDef time;
    char wifi_status;
    
    // 4. 业务数据
    MissedDoseRecord missed_records[4];
    AlarmSche meds_schedule[MAX_SCHE];
    
    // 5. 刷新标志位（用于按需刷新屏幕）
    uint32_t update_flags;

} UI_DisplayData_t;

#endif /* __INFORMATION_H */