#ifndef _BSP_BATTERY_HANDLER_H_
#define _BSP_BATTERY_HANDLER_H_

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_battery_driver.h"

/* 如果有统一的协议头文件，请取消注释并引入 */
// #include "protocol.h" 
// #include "ringbuffer.h"
// extern RingBuffer_t g_ring_buffer;

typedef struct {
    void (*pf_os_delay_ms)(uint32_t const ms);
} bat_handler_os_instance_t;

typedef struct {
    bsp_battery_driver_t      *bat_driver_instance;
    bat_handler_os_instance_t *os_handler_instance;
} bsp_bat_handler_t;

extern bsp_bat_handler_t g_bat_handler_instance;

void battery_handler_task(void *argument);
void bsp_battery_handler_inst(bsp_bat_handler_t *handler, bsp_battery_driver_t *driver, bat_handler_os_instance_t *os);

#endif /* _BSP_BATTERY_HANDLER_H_ */