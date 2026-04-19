#ifndef _BSP_BATTERY_DRIVER_H_
#define _BSP_BATTERY_DRIVER_H_

#include <stdint.h>
#include <stddef.h>

// 分压电路参数 (100K + 100K)
#define RES_UP                 100.0f  
#define RES_DOWN               100.0f  
#define VOLTAGE_DIVIDER_RATIO  ((RES_UP + RES_DOWN) / RES_DOWN) // 2.0
#define VREF_MV                3300U
#define BAT_ADC_MAX            4095U

typedef enum {
    BAT_DRIVER_OK      = 0,
    BAT_DRIVER_ERROR   = 1,
} BAT_DRIVER_Status_t;

/* 硬件接口抽象：只需提供一个读取稳定 ADC 值的函数指针 */
typedef struct {
    uint16_t (*pf_read_stable_adc)(void); 
} bat_hw_interface_t;

/* 驱动对象句柄 */
typedef struct bsp_battery_driver_t {
    bat_hw_interface_t   *hw;

    // 核心方法：获取真实的电池毫伏电压
    BAT_DRIVER_Status_t (*pf_get_voltage_mv)(struct bsp_battery_driver_t *p_driver, uint16_t *voltage_mv);
} bsp_battery_driver_t;

BAT_DRIVER_Status_t bsp_battery_inst(bsp_battery_driver_t *p_driver, bat_hw_interface_t *hw);

#endif /* _BSP_BATTERY_DRIVER_H_ */