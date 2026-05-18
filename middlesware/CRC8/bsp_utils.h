#ifndef _BSP_UTILS_H_
#define _BSP_UTILS_H_

#include <stdint.h>

// 与 ESP32 端完全一致的 CRC8 计算声明
uint8_t bsp_utils_calc_crc8(const uint8_t *data, uint16_t len);

#endif // _BSP_UTILS_H_