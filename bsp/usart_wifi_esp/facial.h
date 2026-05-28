#ifndef __FACIAL_REG_H
#define __FACIAL_REG_H

#include <stdint.h>
#include "facial_reg.h"
/*不定长数据发送*/
// ---------------------------- 协议参数 ----------------------------
#define FR_PACKET_HEAD_VAL       0x5A
#define FR_PACKET_TAIL_VAL       0xFF
#define FR_MAX_PAYLOAD_LEN       32          // 最大数据净荷长度
#define FR_USART_BUFFER_SIZE     64          // 接收缓冲区大小

// ---------------------------- 数据结构 ----------------------------
/* 应用层数据包（不含帧头/帧尾/CRC） */
typedef struct {
    uint8_t sensor_id;                       // 传感器ID
    uint8_t data_len;                        // payload实际长度
    uint8_t payload[FR_MAX_PAYLOAD_LEN];     // 数据净荷
} FR_packet_t;



void FR_USART_Init(void);

void FR_USART_Send(FR_packet_t *packet, uint16_t len);

uint8_t FR_GetPacket(FR_packet_t *packet);

#endif // __FACIAL_REG_H