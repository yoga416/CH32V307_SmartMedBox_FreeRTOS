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


/*LED引脚定义*/
#define LED1_PORT GPIOC
#define LED1_PIN  GPIO_Pin_2
#define LED2_PORT GPIOC
#define LED2_PIN  GPIO_Pin_3
#define LED3_PORT GPIOC
#define LED3_PIN  GPIO_Pin_4

#define LED4_PORT GPIOB
#define LED4_PIN  GPIO_Pin_2

#define LED5_PORT GPIOE
#define LED5_PIN  GPIO_Pin_8
#define LED6_PORT GPIOE
#define LED6_PIN  GPIO_Pin_10
#define LED7_PORT GPIOE
#define LED7_PIN  GPIO_Pin_12
#define LED8_PORT GPIOE
#define LED8_PIN  GPIO_Pin_14

#define LED9_PORT GPIOD
#define LED9_PIN  GPIO_Pin_10

/*蜂鸣器引脚定义*/
#define BUZZER_GPIO_PORT GPIOD
#define BUZZER_PIN GPIO_Pin_11

// 函数声明
void BSP_RTC_Init(void); // 初始化RTC并设置时间
void BSP_RTC_SetDateTime(RTC_TimeTypeDef* time); // 设置时间
void BSP_RTC_GetDateTime(RTC_TimeTypeDef* time); // 获取时间
void BSP_RTC_UpdateNextAlarm(void); // 更新下一个闹钟时间
void BSP_RTC_ModifyTime(uint16_t year, uint16_t month, uint16_t day, uint16_t hour, uint16_t min, uint16_t sec) ; // 修改时间（如果需要单独调用）
void Test_Key_Init(void); // 测试按键初始化

/*LED*/
void LED_Init(void); // LED 初始化函数（测试用）
void LED_Toggle(uint8_t index,uint8_t blink); // LED 翻转函数(blink: 闪烁周期，单位为 100ms，例如 5 就是 500ms)
void LED_STATE(uint8_t index, uint8_t state); // LED 点亮函数(state: 0 关闭，1 打开)


/*BAZZER*/
void Buzzer_Init(void); // 蜂鸣器初始化函数（测试用）
void Buzzer_Toggle(uint8_t blink); // 蜂鸣器翻转函数(blink: 闪烁周期，单位为 100ms，例如 5 就是 500ms)
void Buzzer_STATE(uint8_t state); // 蜂鸣器开启函数(state: 0 关闭，1 打开)

#endif

