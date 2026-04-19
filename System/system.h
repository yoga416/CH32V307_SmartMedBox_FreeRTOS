#ifndef __SYSTEM_H
#define __SYSTEM_H

/* Global define */
#include "ch32v30x.h"
#include "system_config.h"
#include "sht40.h"
#include "usart_wifi_esp.h"
#include "DMA.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"
#include "event_groups.h"
#include "bsp_MLX_90614.h"

/***************** FreeRTOS 相关定义 *****************/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
/***************** 其他系统相关定义 *****************/
#define QUEUE_LENGTH 10

//事件标志位定义
#define EVENT_SENSOR_DATA_READY (1 << 0) // 传感器数据准备就绪事件
#define EVENT_USART_READY (1 << 1) // USART准备就绪事件
#define EVENT_ERROR (1 << 2) // 错误事件
#define EVENT_ALL (EVENT_SENSOR_DATA_READY | EVENT_USART_READY | EVENT_ERROR) // 所有事件



/* Function prototypes */

usart_wifi_esp_Status_t USART_WIFI_ESP_Init(void);
DMA_Status_t USART_DMA_Init(void);
SensorTimStatus_t Sensor_TIM_Init(void);
uint8_t  g_peripheral_init(void);

void UT_Hardware_Verify(void);
uint8_t UT_SHT40_Ping(void);


#endif
