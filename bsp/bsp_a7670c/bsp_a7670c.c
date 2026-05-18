#include "bsp_a7670c.h"
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"

//========================================================
// 底层辅助通信函数
//========================================================
 volatile uint8_t a7670c_ready; // 在A7670C初始化完成后置1
/**
 * @brief  向模组发送字符串
 */
void A7670C_SendString(const char* str) 
{
    while(*str) {
        USART_SendData(A7670C_USART, (uint16_t)*str);
        while (USART_GetFlagStatus(A7670C_USART, USART_FLAG_TXE) == RESET);
        str++;
    }
}

/**
 * @brief  带超时的串口响应等待函数 (FreeRTOS 友好版)
 * @retval 1: 成功收到期望字符串; 0: 超时未收到
 */
uint8_t A7670C_WaitResponse(const char* expected_str, uint32_t timeout_ms) 
{
    char rx_buf[128] = {0}; 
    uint16_t rx_idx = 0;
    uint32_t start_time = xTaskGetTickCount(); 

    // 清空一次残留数据
    USART_ReceiveData(A7670C_USART);

    while ((xTaskGetTickCount() - start_time) < pdMS_TO_TICKS(timeout_ms)) {
        if (USART_GetFlagStatus(A7670C_USART, USART_FLAG_RXNE) != RESET) {
            char c = (char)USART_ReceiveData(A7670C_USART);
            
            if (rx_idx < sizeof(rx_buf) - 1) {
                rx_buf[rx_idx++] = c;
                rx_buf[rx_idx] = '\0'; 
            } else {
                rx_idx = 0; 
                memset(rx_buf, 0, sizeof(rx_buf));
            }

            if (strstr(rx_buf, expected_str) != NULL) {
                return 1; // 匹配成功
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(1)); // 让出 CPU
        }
    }
    
    printf("A7670C response timeout! Expected: %s\r\n", expected_str);
    return 0;
}

//========================================================
// 模组业务功能实现
//========================================================

void A7670C_Init(void) 
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    uint8_t init_success = 0;
    uint8_t total_retries = 3; // 硬件总重启尝试次数

    // 1. 使能时钟
    RCC_APB2PeriphClockCmd(A7670C_PWRKEY_GPIO_CLK | A7670C_USART_TX_GPIO_CLK | A7670C_USART_RX_GPIO_CLK, ENABLE);
    RCC_APB1PeriphClockCmd(A7670C_USART_CLK, ENABLE);
    
    // 2. 引脚配置 (PWRKEY, TX, RX)
    GPIO_InitStructure.GPIO_Pin = A7670C_PWRKEY_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(A7670C_PWRKEY_GPIO_PORT, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = A7670C_USART_TX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(A7670C_USART_TX_GPIO_PORT, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = A7670C_USART_RX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(A7670C_USART_RX_GPIO_PORT, &GPIO_InitStructure);
    
    // 3. USART 配置
    USART_InitStructure.USART_BaudRate = A7670C_BAUDRATE;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(A7670C_USART, &USART_InitStructure);
    USART_Cmd(A7670C_USART, ENABLE);

    // --- 开始带有“硬复位”逻辑的初始化循环 ---
    while (total_retries--) {
       // 【修正后的硬件开机脉冲】
        A7670C_PWRKEY_HIGH();            // 先确保按键是松开的
        vTaskDelay(pdMS_TO_TICKS(100));  // 稍微稳定一下电平
        A7670C_PWRKEY_LOW();             // 按下开机键
        vTaskDelay(pdMS_TO_TICKS(1000)); // 保持按下 1 秒钟 (标准要求 0.5s ~ 1.5s)
        A7670C_PWRKEY_HIGH();            // 【关键】必须松开开机键！
        
        // 等待系统基础启动（A7670C 启动较慢，建议给足时间）
        vTaskDelay(pdMS_TO_TICKS(5000)); 

        // 4. AT 指令同步
        uint8_t at_retry = 3;
        uint8_t handshake_ok = 0;
        while(at_retry--) {
            A7670C_SendString(A7670C_CMD_AT); 
            if (A7670C_WaitResponse(A7670C_RESP_OK, 50)) {
                handshake_ok = 1;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        if (handshake_ok) {
            // 关闭回显
            A7670C_SendString(A7670C_CMD_ECHO_OFF); 
            A7670C_WaitResponse(A7670C_RESP_OK, 1000);

            // Key: Try to set SMS mode, this checks if the module is really ready
            A7670C_SendString(A7670C_CMD_SMS_MODE);
            if (A7670C_WaitResponse(A7670C_RESP_OK, 2000)) {
                init_success = 1;
                a7670c_ready = 1; // 设置全局标志，通知其他任务模块已准备好
                break; // 跳出 total_retries 循环
            } else {
                printf("A7670C service not ready, preparing for hardware restart...\r\n");
            }
        } else {
            printf("A7670C hardware no response, preparing for hardware restart...\r\n");
        }

        // 如果走到这里说明失败了，执行硬关机脉冲，准备下一轮上电
        A7670C_PWRKEY_LOW();
        vTaskDelay(pdMS_TO_TICKS(3000)); // 关机脉冲
        A7670C_PWRKEY_HIGH();
        vTaskDelay(pdMS_TO_TICKS(2000)); // 停顿一下
    }

    if (!init_success) {
        printf("FATAL ERROR: A7670C failed to initialize after multiple restarts!\r\n");
    }
}




/**
 * @brief  将普通手机号字符串转换为 UCS2 格式字符串
 * @example "138" -> "003100330038"
 */
static void Phone_To_UCS2(const char* phone, char* out_ucs2) 
{
    while (*phone) {
        sprintf(out_ucs2, "00%02X", *phone);
        phone++;
        out_ucs2 += 4;
    }
    *out_ucs2 = '\0';
}

/**
 * @brief  将 UTF-8 编码的中文转为 UCS2 格式十六进制字符串
 * @note   这是最简化的 UTF-8 解码逻辑，适用于大多数中文字符
 */
static void UTF8_To_UCS2_String(const char* utf8, char* out_ucs2) 
{
    uint8_t* p = (uint8_t*)utf8;
    while (*p) {
        uint16_t ucs2_val;
        if ((*p & 0xE0) == 0xE0) { // 3字节格式 (标准汉字)
            ucs2_val = ((uint16_t)(p[0] & 0x0F) << 12) | 
                       ((uint16_t)(p[1] & 0x3F) << 6) | 
                       ((uint16_t)(p[2] & 0x3F));
            p += 3;
        } else if ((*p & 0xC0) == 0xC0) { // 2字节格式
            ucs2_val = ((uint16_t)(p[0] & 0x1F) << 6) | 
                       ((uint16_t)(p[1] & 0x3F));
            p += 2;
        } else { // 1字节格式 (ASCII)
            ucs2_val = *p;
            p += 1;
        }
        sprintf(out_ucs2, "%04X", ucs2_val);
        out_ucs2 += 4;
    }
    *out_ucs2 = '\0';
}

/**
 * @brief  全自动中文短信发送接口
 */
/**
 * @brief  全自动中文短信发送接口 (优化版：解决栈溢出)
 */
void A7670C_SendSMS_Auto(const char* raw_phone, const char* raw_text) 
{
    static char phone_ucs2[64];
    static char text_ucs2[512];
    static char cmd_buf[128];

    // 1. Pre-check: clear UART buffer to prevent previous garbled data from interfering with matching
    while(USART_GetFlagStatus(A7670C_USART, USART_FLAG_RXNE) != RESET) {
        USART_ReceiveData(A7670C_USART);
    }

    // 2. Encoding conversion (convert all to UCS2 hex regardless of Chinese or English)
    Phone_To_UCS2(raw_phone, phone_ucs2);
    UTF8_To_UCS2_String(raw_text, text_ucs2);

    // 3. Enter UCS2 mode
    A7670C_SendString("AT+CSCS=\"UCS2\"\r\n");
    if(!A7670C_WaitResponse("OK", 2000)) return;

    // 4. Set SMS parameters
    A7670C_SendString("AT+CSMP=17,167,0,8\r\n");
    if(!A7670C_WaitResponse("OK", 2000)) return;

    // 5. Send number (must be UCS2 format)
    memset(cmd_buf, 0, sizeof(cmd_buf));
    snprintf(cmd_buf, sizeof(cmd_buf), "AT+CMGS=\"%s\"\r\n", phone_ucs2);
    A7670C_SendString(cmd_buf);

    // 6. Wait for prompt (give enough 5 seconds)
    if (A7670C_WaitResponse(">", 5000)) 
    {
        // 7. Send content (UCS2 format)
        A7670C_SendString(text_ucs2);
        
        // Key: delay a bit before sending the end character to ensure data is sent
        vTaskDelay(pdMS_TO_TICKS(200)); 
        
        USART_SendData(A7670C_USART, 0x1A);
        
        
        if(A7670C_WaitResponse("+CMGS", 15000)) {
        } else {
            printf("Operator did not confirm, please check signal or balance\r\n");
        }
    } else {
        printf("No prompt received for SMS content, aborting send\r\n");
        USART_SendData(A7670C_USART, 0x1B); // send ESC
    }

    // 8. Restore to normal mode (important: for next initialization)
    A7670C_SendString("AT+CSCS=\"GSM\"\r\n");
    A7670C_WaitResponse("OK", 1000);
}

void A7670C_PowerOff(void)
{
    // 1. Force switch back to normal mode to ensure the module can understand the shutdown command
    A7670C_SendString("AT+CSCS=\"GSM\"\r\n");
    vTaskDelay(pdMS_TO_TICKS(500));

    // 2. 尝试软件关机
    A7670C_SendString("AT+CPOWD=1\r\n");
    
    // Wait for response
    if (A7670C_WaitResponse("NORMAL POWER DOWN", 5000)) {
    
        vTaskDelay(pdMS_TO_TICKS(3000)); // Give module 3 seconds to turn off
    } 
    else {
        // 3. If software does not respond, perform hardware forced shutdown (hard method)
        printf("No response to software shutdown, starting hardware forced shutdown pulse...\r\n");
        
        A7670C_PWRKEY_HIGH(); 
        vTaskDelay(pdMS_TO_TICKS(100));
        
        A7670C_PWRKEY_LOW();  // pull low
        vTaskDelay(pdMS_TO_TICKS(3500)); // A7670C shutdown pulse usually needs > 2.5s
        
        A7670C_PWRKEY_HIGH(); // release
    }

    // 4. Completely turn off UART to prevent level backflow
    USART_Cmd(A7670C_USART, DISABLE);
    
    // 5. It is recommended to configure TX/RX pins as floating input to completely cut off current to the module
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = A7670C_USART_TX_PIN | A7670C_USART_RX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
    GPIO_Init(A7670C_USART_TX_GPIO_PORT, &GPIO_InitStructure);
}

/**
 * @brief 让模组进入自动睡眠模式 (CSCLK=2)
 */
void A7670C_EnterSleep(void)
{
    // Force switch back to GSM mode to prevent character set confusion
    A7670C_SendString("AT+CSCS=\"GSM\"\r\n");
    vTaskDelay(pdMS_TO_TICKS(100));

    // Enable \"auto sleep mode\"
    A7670C_SendString("AT+CSCLK=2\r\n");
    if(A7670C_WaitResponse("OK", 1000)) {

    }
}

/**
 * @brief 通过串口纯软件唤醒模组
 */
void A7670C_WakeUp(void)
{

    A7670C_SendString("AT\r\n");
    
    vTaskDelay(pdMS_TO_TICKS(150)); 
    
    A7670C_SendString("AT\r\n");
    if(A7670C_WaitResponse("OK", 1000)) {
        
        A7670C_SendString("AT+CSCLK=0\r\n");
        A7670C_WaitResponse("OK", 500);
    } 
    else {
        printf("No response to wakeup, please check wiring or restart.\r\n");
    }
}