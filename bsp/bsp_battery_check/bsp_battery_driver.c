#include "bsp_battery_driver.h"
#include <string.h>

static BAT_DRIVER_Status_t bat_get_voltage_mv(bsp_battery_driver_t *p, uint16_t *voltage_mv) {
    if (!p->hw || !p->hw->pf_read_stable_adc) return BAT_DRIVER_ERROR;

    // 1. 读取底层已经过极高精度滤波的 ADC 值
    uint16_t adc_val = p->hw->pf_read_stable_adc();

    // 2. 转换为实际引脚电压 (先乘后除防精度丢失，强转32位防溢出)
    uint32_t v_pin = (uint32_t)adc_val * VREF_MV / BAT_ADC_MAX;
    
    // 3. 乘以分压比还原电池真实电压
    *voltage_mv = (uint16_t)(v_pin * VOLTAGE_DIVIDER_RATIO);
    
    return BAT_DRIVER_OK;
}

BAT_DRIVER_Status_t bsp_battery_inst(bsp_battery_driver_t *p_driver, bat_hw_interface_t *hw) {
    if (p_driver == NULL || hw == NULL) return BAT_DRIVER_ERROR;
    memset(p_driver, 0, sizeof(bsp_battery_driver_t));
    p_driver->hw = hw;
    p_driver->pf_get_voltage_mv = bat_get_voltage_mv;
    return BAT_DRIVER_OK;
}