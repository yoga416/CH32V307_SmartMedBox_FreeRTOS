#ifndef _BSP_A7670C_H
#define _BSP_A7670C_H

#include "bsp_a7670c_reg.h" // 包含硬件与指令的宏定义

//========================================================
// 对外提供的业务接口
//========================================================

/**
 * @brief  初始化 A7670C 模组及其对应的单片机外设
 * @note   需在 FreeRTOS 调度器启动后调用
 */
void A7670C_Init(void);

/**
 * @brief  发送文本短信
 * @param  phone_number: 目标手机号字符串
 * @param  message: 短信正文内容
 */
void A7670C_SendSMS(const char* phone_number, const char* message);

/**
 * @brief  模组硬件关机
 */
void A7670C_PowerOff(void);

/**
 * @brief  让模组进入睡眠
 */
void A7670C_EnterSleep(void);

/**
 * @brief  唤醒模组
 */
void A7670C_WakeUp(void);

//========================================================
// 底层调试/扩展接口 (如果你需要发其他 AT 指令，可直接调用)
//========================================================
void A7670C_SendString(const char* str);
void A7670C_SendSMS_Auto(const char* raw_phone, const char* raw_text) ;
uint8_t A7670C_WaitResponse(const char* expected_str, uint32_t timeout_ms);

#endif // _BSP_A7670C_H