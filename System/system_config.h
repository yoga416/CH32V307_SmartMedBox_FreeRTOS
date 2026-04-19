#ifndef __SYS_CONFIG_H
#define __SYS_CONFIG_H

// System configuration header file
#include "ch32v30x.h"

/*===============I2C1引脚选择========================*/
/* 1: remap到PB8/PB9, 0: 默认PB6/PB7 */
#define I2C1_USE_REMAP 1

#if I2C1_USE_REMAP
#define I2C1_PORT GPIOB
#define I2C1_SCL_PIN GPIO_Pin_8
#define I2C1_SDA_PIN GPIO_Pin_9
#else
#define I2C1_PORT GPIOB
#define I2C1_SCL_PIN GPIO_Pin_6
#define I2C1_SDA_PIN GPIO_Pin_7
#endif

#define I2C_IT_PORT GPIOC
#define I2C_IT_PIN GPIO_Pin_6
/*===============调试串口配置（USART）引脚配置========================*/
#define DEBUG_USART_PORT GPIOA
#define DEBUG_USART_TX_PIN GPIO_Pin_9
#define DEBUG_USART_RX_PIN GPIO_Pin_10


/*===============数据串口配置（USART）引脚配置========================*/
#define DATA_USART_PORT GPIOA
#define DATA_USART_TX_PIN GPIO_Pin_2
#define DATA_USART_RX_PIN GPIO_Pin_3
/**************** 传感器采集TIM*************************************/
#define SENSOR_TIM TIM2




//TIM2的状态
typedef enum {
    SENSOR_TIM_OK = 0,
    SENSOR_TIM_ERROR = -1,
} SensorTimStatus_t;
#endif // !__SYS_CONFIG_H