#ifndef __SYS_CONFIG_H
#define __SYS_CONFIG_H

#include "ch32v30x.h"

/* =========================================================================
 * 1. I2C 总线引脚配置 (用于 SHT40, MLX90614, MAX30102 等传感器)
 * 注: 硬件上已 Remap 到 PB8 / PB9
 * ========================================================================= */
#define I2C1_PORT           GPIOB
#define I2C1_SCL_PIN        GPIO_Pin_8
#define I2C1_SDA_PIN        GPIO_Pin_9

// I2C 中断引脚 (如果有的传感器用了外部中断提醒)
#define I2C_IT_PORT         GPIOC
#define I2C_IT_PIN          GPIO_Pin_6

/* =========================================================================
 * 2. USART 串口引脚配置
 * ========================================================================= */
// 调试串口 (Printf 打印输出) - USART1 默认
#define DEBUG_USART_PORT    GPIOA
#define DEBUG_USART_TX_PIN  GPIO_Pin_9
#define DEBUG_USART_RX_PIN  GPIO_Pin_10

// 数据/通信串口 (连接 ESP8266 WIFI 模组 / 语音模组) - USART2 重映射
#define DATA_USART_PORT     GPIOD
#define DATA_USART_TX_PIN   GPIO_Pin_5
#define DATA_USART_RX_PIN   GPIO_Pin_6

//天问通信串口 (由于 PB10/PB11 被触摸屏占用，使用完全重映射至 PD8/PD9) - USART3
#define TW_USART_PORT       GPIOD
#define TW_USART_TX_PIN     GPIO_Pin_8
#define TW_USART_RX_PIN     GPIO_Pin_9

//触摸屏通信串口 - USART4 默认
#define TS_USART_PORT       GPIOC
#define TS_USART_TX_PIN     GPIO_Pin_10
#define TS_USART_RX_PIN     GPIO_Pin_11

//人脸识别串口 - USART5 默认
#define FR_USART_PORT       GPIOC
#define FR_USART_TX_PIN     GPIO_Pin_12
#define FR_USART_RX_PIN     GPIO_Pin_2

/* =========================================================================
 * 3. 硬件定时器配置
 * ========================================================================= */
// 传感器基础采集/心跳滴答定时器
#define SENSOR_TIM          TIM2

// TIM2 状态枚举
typedef enum {
    SENSOR_TIM_OK = 0,
    SENSOR_TIM_ERROR = -1,
} SensorTimStatus_t;

#endif /* __SYS_CONFIG_H */