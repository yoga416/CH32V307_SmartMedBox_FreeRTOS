#include "Middle_ring_buffer.h"
#include <string.h>

// 初始化环形缓冲区
void RingBuffer_Init(RingBuffer_t *rb) {
    if (rb == NULL) return;
    rb->head = 0;
    rb->tail = 0;
    memset(rb->buffer, 0, sizeof(rb->buffer));
}

// 检查环形缓冲区是否为空
int RingBuffer_isEmpty(const RingBuffer_t *rb) {
    return (rb->head == rb->tail) ? 0xAF : 0x00;
}

// 检查环形缓冲区是否已满
int RingBuffer_isFull(const RingBuffer_t *rb) {
    return ((rb->head + 1) % RING_BUFFER_SIZE == rb->tail) ? 0xAF : 0x00;
}

// 将纯净数据包写入环形缓冲区
int RingBuffer_push(RingBuffer_t *rb, Packet_t *packet) {
    if (rb == NULL || packet == NULL) return 0x00;
    if (RingBuffer_isFull(rb) == 0xAF) return 0x00;
    
    // 直接拷贝整个结构体（包含 ID, Len 和 Payload）
    rb->buffer[rb->head] = *packet;
    rb->head = (rb->head + 1) % RING_BUFFER_SIZE;
    return 0xAF;
}

// 从环形缓冲区读取数据包
int RingBuffer_pop(RingBuffer_t *rb, Packet_t *packet) {
    if (rb == NULL || packet == NULL) return 0x00;
    if (RingBuffer_isEmpty(rb) == 0xAF) return 0x00;
    
    *packet = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1) % RING_BUFFER_SIZE;
    return 0xAF;
}