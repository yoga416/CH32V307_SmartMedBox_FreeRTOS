#ifndef _MIDDLE_RING_BUFFER_H
#define _MIDDLE_RING_BUFFER_H
#include <stdint.h>
#include "sht40.h"

#define RING_BUFFER_SIZE 256// 环形缓冲区大小，必须是2的幂次方
#define RING_BUFFER_MASK (RING_BUFFER_SIZE - 1) 

#define     PACKET_HEAD                  0x5A       //定义包头和包尾的固定值
#define     PACKET_TAIL                  0xFF       //定义包头和包尾的固定值
#define     SENSOR_DATA_SIZE                6    //每个数据包中包含的传感器数据数量（温度和湿度）

//传感器id定义
#define     SENSOR_ID_TEMPERATURE         0x00
#define     SENSOR_ID_HUMIDITY            0x01
#define     SENSOR_ID_MLX_AMBIENT_TEMP    0x02
#define     SENSOR_ID_MLX_OBJECT_TEMP     0x03
#define     SENSOR_ID_HEART_RATE          0x04
#define     SENSOR_ID_SPO2                0x05
//sensor data structure//每个传感器数据包含16位sensor_id和32位data（小端发送）
typedef struct __attribute__((packed)) {
   uint16_t sensor_id;//传感器ID
    uint16_t data;     //传感器数据值（32位）
} SensorData_t;

//packet structure//按字节顺序：head(1) length(1) sensor_num(1) sensor_data[] crc(1) tail(1)
typedef struct {
    uint8_t head[1];
    uint8_t length; // 包长度字段，表示sensor_data数组中有效数据的字节数
    uint8_t sensor_num;
    SensorData_t sensor_data[SENSOR_DATA_SIZE];
    uint8_t crc;
    uint8_t tail[1];
} __attribute__((packed)) Packet_t;

typedef struct {
    Packet_t buffer[RING_BUFFER_SIZE];
    uint16_t head;
    uint16_t tail;
} RingBuffer_t;

//初始化环形缓冲区
void RingBuffer_Init(RingBuffer_t *rb) ;
//将数据包写入环形缓冲区
int RingBuffer_push(RingBuffer_t *rb,  Packet_t *packet) ;
//从环形缓冲区读取数据包
int RingBuffer_pop(RingBuffer_t *rb, Packet_t *packet) ;
//检查环形缓冲区是否为空
int RingBuffer_isEmpty( const RingBuffer_t *rb) ;
//检查环形缓冲区是否已满
int RingBuffer_isFull(const RingBuffer_t *rb) ;
//CRC校验函数
uint8_t Calculate_CRC(const Packet_t *packet) ;


#endif // !_MIDDLE_RING_BUFFER_H
