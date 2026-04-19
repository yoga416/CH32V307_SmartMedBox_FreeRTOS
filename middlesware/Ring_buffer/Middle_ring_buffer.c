#include "Middle_ring_buffer.h"

//头文件定义
#include<stdint.h>
#include<stdio.h>
#include "string.h"

//函数声明
int RingBuffer_isFull(const RingBuffer_t *rb);
int RingBuffer_isEmpty(const RingBuffer_t *rb);
/*
*@brief: 初始化环形缓冲区
*@param: rb - 环形缓冲区指针
*@return: 无
*/
void RingBuffer_Init(RingBuffer_t *rb) {
    rb->head = 0;
    rb->tail = 0;
}

/**
 * @brief: 将数据包写入环形缓冲区
 * @param: rb - 环形缓冲区指针
 * @param: packet - 要写入的数据包指针
 * @param: length - 要写入的数据长度（字节）
 * @return: 0xAF表示成功，0xFF表示失败
 */
int RingBuffer_push(RingBuffer_t *rb, Packet_t *packet)
{
    // 1. 基础指针检查
    if (rb == NULL || packet == NULL)
    {
        return 0xFF; 
    }
    // 2. 检查缓冲区是否已满
    if (RingBuffer_isFull(rb) == 0xFF)
    {
        return 0xFF; 
    }
    // 3. 写入数据
    rb->buffer[rb->head] = *packet;
    rb->head = (rb->head + 1) & RING_BUFFER_MASK; // 更新头指针
    return 0xAF; // 成功
}
/*
*@brief: 检查环形缓冲区是否已满
*@param: rb - 环形缓冲区指针
*@return: 0xFF表示已满，0xAF表示未满
*/
int RingBuffer_isFull(const RingBuffer_t *rb)
{
      if(rb==NULL)
    {
        return 0xFF; // 参数错误
    }
    if (((rb->head + 1) & RING_BUFFER_MASK) == rb->tail)
    {
        return 0xFF; // 已满
    }
    return 0xAF; // 未满
}

/*
*@brief: 检查环形缓冲区是否为空
*@param: rb - 环形缓冲区指针
*@return: 0xFF表示为空，0xAF表示非空
*/
int RingBuffer_isEmpty(const RingBuffer_t *rb)
{
    if(rb==NULL)
    {
        return 0xFF; // 参数错误，视为为空
    }
      if(rb->head==rb->tail)
      {
            return 0xFF; // 为空
      }
      return 0xAF; // 非空
}
/*
*@brief: 从环形缓冲区读取数据包
*@param: rb - 环形缓冲区指针
*@param: packet - 要读取的数据包指针
*@param: length - 要读取的数据长度（字节）
*@return: 0xAF表示成功，0xFF表示失败
*/
int RingBuffer_pop(RingBuffer_t *rb, Packet_t *packet)
{
    if(rb == NULL || packet == NULL) {
        return 0xFF; // 参数错误
    }
      if (RingBuffer_isEmpty(rb) == 0xFF) {
            return 0xFF; // 缓冲区为空
      }
            *packet = rb->buffer[rb->tail];
      rb->tail = (rb->tail + 1) & RING_BUFFER_MASK; // 更新尾指针
    return 0xAF; // 成功
}
//CRC校验函数
uint8_t Calculate_CRC(const Packet_t *packet) {
      if (packet == NULL) {
            return 0xFF; // 参数错误
      }
    uint8_t crc = 0x00; // CRC-8, poly=0x31, init=0x00, xorout=0x00
      const uint8_t *p = (const uint8_t *)packet;
      for (size_t i = 0; i < sizeof(Packet_t) - 2; i++) { // 不包括crc和tail
        crc ^= p[i];
        for (uint8_t b = 0; b < 8; b++) {
            if (crc & 0x80) {
                crc = (uint8_t)((crc << 1) ^ 0x31);
            } else {
                crc <<= 1;
            }
        }
      }
      return crc; 
}
