#ifndef _BSP_A7670C_REG_H
#define _BSP_A7670C_REG_H


//USARTd define
#define A7670C_USART                 USART3
#define A7670C_USART_CLK             RCC_APB1Periph_USART3

#define A7670C_USART_TX_PIN          GPIO_Pin_10
#define A7670C_USART_TX_GPIO_PORT    GPIOB
#define A7670C_USART_TX_GPIO_CLK     RCC_APB2Periph_GPIOB

#define A7670C_USART_RX_PIN          GPIO_Pin_11
#define A7670C_USART_RX_GPIO_PORT    GPIOB
#define A7670C_USART_RX_GPIO_CLK     RCC_APB2Periph_GPIOB

//PWRKEY引脚定义
#define A7670C_PWRKEY_PIN             GPIO_Pin_5
#define A7670C_PWRKEY_GPIO_PORT       GPIOB
#define A7670C_PWRKEY_GPIO_CLK        RCC_APB2Periph_GPIOB

// 引脚操作宏
#define A7670C_PWRKEY_HIGH()          GPIO_SetBits(A7670C_PWRKEY_GPIO_PORT, A7670C_PWRKEY_PIN)
#define A7670C_PWRKEY_LOW()           GPIO_ResetBits(A7670C_PWRKEY_GPIO_PORT, A7670C_PWRKEY_PIN)

//BAUDRATE
#define A7670C_USART_BAUDRATE_115200      115200U

//指令集
#define A7670C_AT_CMD                     "AT\r\n"
#define A7670C_AT_CMD_OK_RESPONSE         "OK\r\n"

/* 基础查询指令 */
#define A7670C_CMD_CPIN          "AT+CPIN?\r\n"      // 查询SIM卡状态
#define A7670C_CMD_CSQ           "AT+CSQ\r\n"        // 查询信号强度
#define A7670C_CMD_CEREG         "AT+CEREG?\r\n"     // 查询4G注册状态

/* 语音通话相关 */
#define A7670C_CMD_CALL_DIAL     "ATD%s;\r\n"        // 拨号格式化字符串
#define A7670C_CMD_CALL_HANGUP   "ATH\r\n"           // 挂断
#define A7670C_CMD_CALL_ANSWER   "ATA\r\n"           // 接听

/* 短信相关 */
#define A7670C_CMD_SMS_MODE      "AT+CMGF=1\r\n"     // 设置为TEXT模式
#define A7670C_CMD_SMS_SEND      "AT+CMGS=\"%s\"\r\n"// 发送短信格式化

/* 预期响应标记 (用于解析) */
#define A7670C_RESP_OK           "OK"
#define A7670C_RESP_ERROR        "ERROR"
#define A7670C_RESP_READY        "+CPIN: READY"
#define A7670C_RESP_CEREG_DONE   "+CEREG: 0,1"       // 注册成功标识
#define A7670C_RESP_RING         "RING"              // 振铃主动上报
#define A7670C_RESP_SMS_PROMPT   "> "                // 短信输入提示


#define A7670C_CMD_ECHO_OFF    "ATE0\r\n"            // 关闭回显，简化响应解析

#endif // _BSP_A7670C_REG_H