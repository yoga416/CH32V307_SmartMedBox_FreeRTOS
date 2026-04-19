/******************************************************************************
 *
 * @file sht40_port.h
 * @brief FreeRTOS porting layer for SHT40 Handler.
 *
 *****************************************************************************/
#ifndef SHT40_PORT_H_
#define SHT40_PORT_H_

#include "sht40_handler.h"

/* 暴露给外部应用层的 FreeRTOS 接口实例 */
extern th_handler_os_interface_t g_sht40_freertos_if;
void on_SHT40_DATA_ready(float * temp ,float *humi);
#endif /* SHT40_PORT_H_ */