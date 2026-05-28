#ifndef _BSP_RTC_REG_H_
#define _BSP_RTC_REG_H_
#include "bsp_rtc.h"

#include "ch32v30x.h"

// 定义一个时间结构体
typedef struct {
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  min;
    uint8_t  sec;
} RTC_TimeTypeDef;

// 函数声明
void BSP_RTC_Init(void); // 初始化RTC并设置时间
void BSP_RTC_SetDateTime(RTC_TimeTypeDef* time); // 设置时间
void BSP_RTC_GetDateTime(RTC_TimeTypeDef* time); // 获取时间
void BSP_RTC_UpdateNextAlarm(void); // 更新下一个闹钟时间
void BSP_RTC_ModifyTime(uint16_t year, uint16_t month, uint16_t day, uint16_t hour, uint16_t min, uint16_t sec) ; // 修改时间（如果需要单独调用）
void Test_Key_Init(void); // 测试按键初始化
#endif

