#ifndef __BSP_TIANWEN_H
#define __BSP_TIANWEN_H

#include <stdint.h>
#include "bsp_tianwen_reg.h"

/* 发送标准无数据指令 (格式: 5A CMD D5 5A FF) */
void Tianwen_Send_Standard_Cmd(uint8_t cmd);
void Tianwen_Send_Data_Cmd(uint8_t cmd, uint16_t sensor_data);
void Tianwen_Send_Data_Cmd_u8(uint8_t cmd, uint8_t data_high, uint8_t data_low);
void Tianwen_Send_Data_u32(uint8_t cmd, uint32_t sensor_data);

#endif /* __BSP_TIANWEN_H */