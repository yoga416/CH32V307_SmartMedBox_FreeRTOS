#include "bsp_utils.h"

// 多项式: 0x07 (x^8 + x^2 + x + 1), 初始值: 0x00
uint8_t bsp_utils_calc_crc8(const uint8_t *data, uint16_t len) 
{
    uint8_t crc = 0x00;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}