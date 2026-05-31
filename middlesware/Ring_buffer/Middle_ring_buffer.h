#ifndef _MIDDLE_RING_BUFFER_H
#define _MIDDLE_RING_BUFFER_H
#include <stdint.h>

#define RING_BUFFER_SIZE 64
#define RING_BUFFER_MASK (RING_BUFFER_SIZE - 1) 

// 匹配 ESP32 端的固定值
#define PACKET_HEAD_VAL   0x5A       
#define PACKET_TAIL_VAL   0xFF       
#define MAX_PAYLOAD_LEN   32         // 允许的变长数据最大长度

// 传感器id定义
#define SENSOR_ID_TEMPERATURE_HUMIDITY          0x00
#define SENSOR_ID_MLX_AMBIENT_TEMP              0x01
#define SENSOR_ID_MLX_OBJECT_TEMP               0x02
#define SENSOR_ID_MLX_TEMP_BOTH                 0x03 // MLX90614 同时发送环境温度和物体温度
#define SENSOR_ID_HEART_RATE_SPO2               0x04 // 建议心率和血氧打包成一个结构发送
#define CMD_UPLOAD_MISSED_MED                   0x20 // 新增：漏服记录上传

// 新版应用层数据包结构 (存入 RingBuffer 的纯净数据，不含帧头尾)
typedef struct {
    uint8_t sensor_id;                 // 传感器ID
    uint8_t user_id;                   // 用户ID，预留字段，当前版本固定为0
    uint8_t data_len;                  // payload 实际有效长度
    uint8_t payload[MAX_PAYLOAD_LEN];  // 数据净荷区
} Packet_t;

typedef struct {
    Packet_t buffer[RING_BUFFER_SIZE];
    uint16_t head;
    uint16_t tail;
} RingBuffer_t;

// API 声明保持不变
void RingBuffer_Init(RingBuffer_t *rb);
int RingBuffer_push(RingBuffer_t *rb, Packet_t *packet);
int RingBuffer_pop(RingBuffer_t *rb, Packet_t *packet);
int RingBuffer_isEmpty(const RingBuffer_t *rb);
int RingBuffer_isFull(const RingBuffer_t *rb);

#endif // !_MIDDLE_RING_BUFFER_H