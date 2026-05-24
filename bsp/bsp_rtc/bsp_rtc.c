#include "bsp_rtc.h"
#include <time.h>
#include "information.h" // 引入全局数据结构定义
// 定义吃药时间表
extern AlarmSche my_meds[3]; // 从 bsp_rtc.h 中 extern 引入吃药时间表

void BSP_RTC_Init(void) 
{
    // 1. 开启电源、备份域、以及重要的 EXTI 时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
    // 注意：有些工程需要开启 RCC_APB2Periph_AFIO 才能使用 EXTI
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); 
    
    PWR_BackupAccessCmd(ENABLE);

    // 2. 始终开启 LSI
    RCC_LSICmd(ENABLE);
    while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET);

    // 3. 判断是否需要执行“第一次”配置
    if (BKP_ReadBackupRegister(BKP_DR1) != 0xA5A5) 
    
    {

        RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);
        RCC_RTCCLKCmd(ENABLE);

        RTC_WaitForSynchro();
        RTC_WaitForLastTask();

        RTC_EnterConfigMode();
        RTC_SetPrescaler(40000 - 1); 
        RTC_WaitForLastTask(); 
        
        RTC_EnterConfigMode();
        RTC_ITConfig(RTC_IT_SEC | RTC_IT_ALR, ENABLE);
        RTC_ExitConfigMode();
        RTC_WaitForLastTask();

        RTC_TimeTypeDef initTime = {2026, 3, 2, 00, 0, 0};
        BSP_RTC_SetDateTime(&initTime);

        // 设置初始闹钟（注意：设置闹钟最好也放在 ConfigMode 里，或者确保前面 Exit 了）
        RTC_EnterConfigMode();
        BSP_RTC_UpdateNextAlarm(); // 设置第一个闹钟
        RTC_ExitConfigMode();
        RTC_WaitForLastTask();

        BKP_WriteBackupRegister(BKP_DR1, 0xA5A5);
    }
    else 
    {
        RTC_WaitForSynchro();
        RTC_WaitForLastTask();
        
        // 重新使能 RTC 内部中断
        RTC_EnterConfigMode();
        RTC_ITConfig(RTC_IT_SEC | RTC_IT_ALR, ENABLE);
        RTC_ExitConfigMode();
        RTC_WaitForLastTask();
    }

    // --- 【新增部分】 4. EXTI 线 17 配置 ---
    // 闹钟中断必须通过 EXTI Line 17 才能触发 NVIC
    EXTI_InitTypeDef EXTI_InitStructure;
    EXTI_ClearITPendingBit(EXTI_Line17);
    EXTI_InitStructure.EXTI_Line = EXTI_Line17;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; // 上升沿触发
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    // 5. NVIC 配置
    NVIC_SetPriority(RTC_IRQn, 5);
    NVIC_EnableIRQ(RTC_IRQn);
    
    RTC_ClearFlag(RTC_FLAG_SEC | RTC_FLAG_ALR);
}
static const uint8_t month_days[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

static inline int is_leap_year(uint16_t year) {
    return ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0));
}

static uint32_t custom_mktime(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec) {
    uint32_t days = 0;
    for (uint16_t y = 1970; y < year; y++) {
        days += is_leap_year(y) ? 366 : 365;
    }
    for (uint8_t m = 1; m < month; m++) {
        days += month_days[m - 1];
        if (m == 2 && is_leap_year(year)) days++;
    }
    days += day - 1;
    return days * 86400 + hour * 3600 + min * 60 + sec;
}

static void custom_localtime(uint32_t timestamp, RTC_TimeTypeDef *t) {
    uint32_t days = timestamp / 86400;
    uint32_t secs = timestamp % 86400;
    
    t->hour = secs / 3600;
    t->min = (secs % 3600) / 60;
    t->sec = secs % 60;
    
    uint16_t year = 1970;
    while (1) {
        uint32_t days_in_year = is_leap_year(year) ? 366 : 365;
        if (days >= days_in_year) {
            days -= days_in_year;
            year++;
        } else {
            break;
        }
    }
    t->year = year;
    
    uint8_t month = 1;
    while (1) {
        uint32_t days_in_month = month_days[month - 1];
        if (month == 2 && is_leap_year(year)) days_in_month++;
        
        if (days >= days_in_month) {
            days -= days_in_month;
            month++;
        } else {
            break;
        }
    }
    t->month = month;
    t->day = days + 1;
}

void BSP_RTC_SetDateTime(RTC_TimeTypeDef* time) {
    uint32_t time_stamp = custom_mktime(time->year, time->month, time->day, time->hour, time->min, time->sec);
    // 写入 RTC 计数器
    PWR_BackupAccessCmd(ENABLE);// 确保备份域访问权限
    RTC_SetCounter(time_stamp);// 写入秒数到 RTC
    RTC_WaitForLastTask();// 等待写入完成
}

/**
 * @brief  获取当前时间并转为年月日结构体
 */
void BSP_RTC_GetDateTime(RTC_TimeTypeDef* time) {
    uint32_t seconds = RTC_GetCounter();// 从 RTC 获取秒数
    custom_localtime(seconds, time);
}


 void BSP_RTC_UpdateNextAlarm(void) {
    RTC_TimeTypeDef now;
    BSP_RTC_GetDateTime(&now); // 获取当前的年月日时分秒

    uint32_t current_stamp = RTC_GetCounter();
    uint32_t min_next_stamp = 0xFFFFFFFF; 

    for (int i = 0; i < MAX_SCHE; i++) {
        uint32_t target_stamp = custom_mktime(now.year, now.month, now.day, my_meds[i].hour, my_meds[i].min, 0);

        // 如果这个时间点今天已经过去了，就设为明天的这个时间
        if (target_stamp <= current_stamp) {
            target_stamp += 86400; // 加一天的秒数
        }

        // 寻找距离现在最近的那一个
        if (target_stamp < min_next_stamp) {
            min_next_stamp = target_stamp;
        }
    }

    // 写入硬件寄存器
    RTC_EnterConfigMode();
    RTC_SetAlarm(min_next_stamp);
    RTC_ExitConfigMode();
    RTC_WaitForLastTask();
    }

/**
 * @brief  供 app_task 或 UI 调用的修改时间函数
 */
void BSP_RTC_ModifyTime(uint16_t year, uint16_t month, uint16_t day, uint16_t hour, uint16_t min, uint16_t sec) 
{
    // 1. 构造新的时间结构体
    RTC_TimeTypeDef new_time = {year, month, day, hour, min, sec};
    
    // 2. 写入新的时间到硬件
    BSP_RTC_SetDateTime(&new_time);
    
    // 3. 【极其重要】时间被强行改变了，必须重新计算下一个吃药的闹钟时间！
    // 否则闹钟会错乱或者根本不响
    BSP_RTC_UpdateNextAlarm();
}