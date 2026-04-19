#ifndef _BSP_MAX30102_PORT_H_
#define _BSP_MAX30102_PORT_H_

#include "bsp_max_30102_driver.h"
#include "FreeRTOS.h"
#include "semphr.h"

extern bsp_max30102_driver_t g_max30102;

/* 暴露给任务唤醒的引脚中断信号量 (DMA 信号量已删除) */
extern SemaphoreHandle_t xSem_MAX30102_Exti;

void bsp_max30102_port_init(void);
void on_hr_spo2_calculated(int32_t hr, int8_t hr_valid, float spo2, int8_t spo2_valid) ;
void bsp_max30102_handler_init(void) ;
#endif /* _BSP_MAX30102_PORT_H_ */