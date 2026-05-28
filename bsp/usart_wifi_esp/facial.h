#ifndef __FACIAL_H
#define __FACIAL_H

#include "ch32v30x.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "facial_reg.h" // 包含协议相关宏，如FR_PACKET_HEAD_VAL, FR_MAX_PAYLOAD_LEN等

// ---------------- 硬件引脚及资源配置定义 ----------------
#define FR_USART                UART5
#define FR_USART_RCC            RCC_APB1Periph_UART5
#define FR_USART_BAUDRATE       115200
#define FR_USART_IRQn           UART5_IRQn
#define FR_USART_IRQHandler     UART5_IRQHandler

#define FR_USART_TX_PORT        GPIOC
#define FR_USART_TX_PIN         GPIO_Pin_12
#define FR_USART_TX_RCC         RCC_APB2Periph_GPIOC

#define FR_USART_RX_PORT        GPIOD
#define FR_USART_RX_PIN         GPIO_Pin_2
#define FR_USART_RX_RCC         RCC_APB2Periph_GPIOD

// --- DMA配置 (注意: 具体通道请参考CH32V307数据手册中UART5 TX的DMA映射) ---
// 假设 UART5 TX 映射在 DMA2_Channel4
#define FR_TX_DMA_RCC           RCC_AHBPeriph_DMA2
#define FR_TX_DMA_CHANNEL       DMA2_Channel4
#define FR_TX_DMA_FLAG_TC       DMA2_FLAG_TC4
#define FR_TX_DMA_IRQn          DMA2_Channel4_IRQn
#define FR_TX_DMA_IRQHandler    DMA2_Channel4_IRQHandler

// ---------------- FreeRTOS 资源 ----------------
#define FR_RX_QUEUE_LENGTH      256  // 接收队列深度

typedef enum {
    FR_OK = 0,
    FR_ERR_PARAM,
    FR_ERR_TIMEOUT,
    FR_ERR_BUSY
} fr_status_t;

typedef struct {
    uint8_t sensor_id;
    uint8_t data_len;
    uint8_t payload[FR_MAX_PAYLOAD_LEN];
} FR_packet_t;
// ---------------- API 函数声明 ----------------
void FR_Init(void);
fr_status_t FR_SendPacket(FR_packet_t *packet);
fr_status_t FR_ReceivePacket_Block(FR_packet_t *packet, uint32_t timeout_ms);

#endif /* __FACIAL_H */