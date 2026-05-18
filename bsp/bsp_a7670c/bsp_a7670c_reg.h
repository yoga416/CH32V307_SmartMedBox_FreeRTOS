#ifndef _BSP_A7670C_REG_H
#define _BSP_A7670C_REG_H

#include "ch32v30x.h"

//========================================================
// 串口定义 (PC10/PC11 默认复用为 UART4，无需 AFIO 重映射)
//========================================================
#define A7670C_USART                 UART4
#define A7670C_USART_CLK             RCC_APB1Periph_UART4

#define A7670C_USART_TX_PIN          GPIO_Pin_10
#define A7670C_USART_TX_GPIO_PORT    GPIOC
#define A7670C_USART_TX_GPIO_CLK     RCC_APB2Periph_GPIOC

#define A7670C_USART_RX_PIN          GPIO_Pin_11
#define A7670C_USART_RX_GPIO_PORT    GPIOC
#define A7670C_USART_RX_GPIO_CLK     RCC_APB2Periph_GPIOC

//========================================================
// PWRKEY 引脚定义 (用于硬件开机)
//========================================================
#define A7670C_PWRKEY_PIN             GPIO_Pin_5
#define A7670C_PWRKEY_GPIO_PORT       GPIOB
#define A7670C_PWRKEY_GPIO_CLK        RCC_APB2Periph_GPIOB

// 引脚操作宏
#define A7670C_PWRKEY_HIGH()          GPIO_SetBits(A7670C_PWRKEY_GPIO_PORT, A7670C_PWRKEY_PIN)
#define A7670C_PWRKEY_LOW()           GPIO_ResetBits(A7670C_PWRKEY_GPIO_PORT, A7670C_PWRKEY_PIN)

// 设置字符集为 UCS2（发送中文必须）
#define A7670C_CMD_CHARSET_UCS2    "AT+CSCS=\"UCS2\"\r\n"
// 设置短信参数：FO=17, VP=167, PID=0, DCS=8 (8代表UCS2编码)
#define A7670C_CMD_SMS_PARAM       "AT+CSMP=17,167,0,8\r\n"


#define A7670C_CMD_SLEEP_EN     "AT+CSCLK=1\r\n"  // 开启睡眠模式
#define A7670C_CMD_WAKEUP       "AT\r\n"          // 唤醒指令

// 配置参数
#define A7670C_BAUDRATE               115200U

//========================================================
// 短信发送核心 AT 指令
//========================================================
#define A7670C_CMD_AT                "AT\r\n"
#define A7670C_CMD_ECHO_OFF          "ATE0\r\n"      // 关闭回显

/* 短信步骤：1. 设置模式 -> 2. 输入号码 -> 3. 输入内容 -> 4. 发送 0x1A */
#define A7670C_CMD_SMS_MODE          "AT+CMGF=1\r\n"      // 设置为 TEXT 模式
#define A7670C_CMD_SMS_SEND          "AT+CMGS=\"%s\"\r\n" // 拨号格式化字符串

//========================================================
// 核心结束符：CTRL+Z (十六进制 0x1A)
//========================================================
#define A7670C_SMS_END_CHAR          ((uint8_t)0x1A) 

//========================================================
// 预期响应 (用于简单的字符串判断)
//========================================================
#define A7670C_RESP_OK               "OK"
#define A7670C_RESP_ERROR            "ERROR"
#define A7670C_RESP_PROMPT           "> "  // 发送号码后模组返回此符号，表示可以输入短信内容

#endif // _BSP_A7670C_REG_H