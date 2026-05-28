#ifndef __FACIAL_REG_H
#define __FACIAL_REG_H

#include <stdio.h>
#include <stdint.h>
#include <string.h>



#define FR_USART_BAUDRATE 115200
// ---------------------------- 协议参数 ----------------------------
#define FR_PACKET_HEAD_VAL       0x5A
#define FR_PACKET_TAIL_VAL       0xFF
#define FR_MAX_PAYLOAD_LEN       32          // 最大数据净荷长度
#define FR_USART_BUFFER_SIZE     64          // 接收缓冲区大小

/*数据帧格式为*/
/* [帧头] [传感器ID] [数据长度] [数据净荷] [CRC] [帧尾] */

#endif // __FACIAL_REG_H