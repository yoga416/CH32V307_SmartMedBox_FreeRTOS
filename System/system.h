#ifndef __SYSTEM_H
#define __SYSTEM_H

#include "ch32v30x.h"
#include "system_config.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "event_groups.h"

/* =============== 系统状态码定义 =============== */
#define SYSTEM_INIT_OK      0x32
#define SYSTEM_INIT_FAIL    0xFF

/* =============== 暴露给全局的句柄 =============== */
// 如果其他文件（比如传感器驱动）需要用到 I2C 互斥锁或事件组，直接包含 system.h 即可
extern SemaphoreHandle_t xI2CBusMutex;
extern EventGroupHandle_t xEventGroup;

/* =============== 全局外设初始化接口 =============== */
uint8_t g_peripheral_init(void);

#endif /* __SYSTEM_H */