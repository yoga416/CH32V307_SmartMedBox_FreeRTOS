/*
 * File: usart_wifi_esp.h
 * Brief: ESP8266 UART/DMA communication interface definitions.
 */

#ifndef __USART_WIFI_ESP_H
#define __USART_WIFI_ESP_H

#include <stdint.h>
#include "sht40.h"
#include "Middle_ring_buffer.h"

// USART与ESP8266通信相关定义
#define USART_WIFI_ESP_BUFFER_SIZE 20
#define USART_WIFI_ESP_BAUDRATE 115200
#define USART_WIFI_ESP_TIMEOUT_MS 1000
// USART状态枚举
typedef enum{
      USART_WIFI_ESP_OK                = 0,
      USART_WIFI_ESP_ERROR             = -1,
      usart_wifi_esp_busy              = 1,
} 
usart_wifi_esp_Status_t;
// USART通信状态枚举
#if 1
//define usart data structure           
usart_wifi_esp_Status_t UART2_DMA_Start_Send(uint8_t *pbuf, uint16_t len);
usart_wifi_esp_Status_t USART_WIFI_ESP_Send(const Packet_t *packet);
usart_wifi_esp_Status_t USART_WIFI_ESP_Receive(Packet_t *packet);
#endif
#endif // !1 _USART_&WIFI_ESP.
