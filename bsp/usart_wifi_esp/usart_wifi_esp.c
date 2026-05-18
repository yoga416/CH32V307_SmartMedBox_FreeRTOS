#include "usart_wifi_esp.h"
#include "ch32v30x.h"
#include "ch32v30x_usart.h"
#include "ch32v30x_dma.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "bsp_utils.h" 
#include <string.h>

extern SemaphoreHandle_t xDMA_Sem;

// 显式声明中断服务函数
void DMA1_Channel7_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

// 内部 DMA 触发函数
usart_wifi_esp_Status_t UART2_DMA_Start_Send(uint8_t *pbuf, uint16_t len)
{
    if (pbuf == NULL || len == 0) return USART_WIFI_ESP_ERROR;

    DMA_Cmd(DMA1_Channel7, DISABLE);
    DMA1_Channel7->MADDR = (uint32_t)pbuf;
    DMA_SetCurrDataCounter(DMA1_Channel7, len);
    USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
    DMA_Cmd(DMA1_Channel7, ENABLE);

    return USART_WIFI_ESP_OK;
}

// 核心组包与变长发送逻辑
usart_wifi_esp_Status_t USART_WIFI_ESP_Send(const Packet_t *packet)
{
    if (packet == NULL) return USART_WIFI_ESP_ERROR;

    // 静态缓冲区，防止 DMA 传输时内存失效
    static uint8_t tx_dma_buffer[MAX_PAYLOAD_LEN + 5]; 
    uint16_t tx_len = 0;

    tx_dma_buffer[tx_len++] = PACKET_HEAD_VAL; // 0x5A
    tx_dma_buffer[tx_len++] = packet->data_len;
    tx_dma_buffer[tx_len++] = packet->sensor_id;
    
    if (packet->data_len > 0 && packet->data_len <= MAX_PAYLOAD_LEN) {
        memcpy(&tx_dma_buffer[tx_len], packet->payload, packet->data_len);
        tx_len += packet->data_len;
    }

    // 计算 CRC8 (从 Length 开始校验)
    tx_dma_buffer[tx_len] = bsp_utils_calc_crc8(&tx_dma_buffer[1], tx_len - 1);
    tx_len++;

    tx_dma_buffer[tx_len++] = PACKET_TAIL_VAL; // 0xFF

    UART2_DMA_Start_Send(tx_dma_buffer, tx_len);

    if (xSemaphoreTake(xDMA_Sem, pdMS_TO_TICKS(100)) == pdTRUE) {
        return USART_WIFI_ESP_OK;
    } else {
        DMA_Cmd(DMA1_Channel7, DISABLE);
        return USART_WIFI_ESP_ERROR;
    }
}

// DMA 中断回调
void DMA1_Channel7_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_IT_TC7) != RESET) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        DMA_ClearITPendingBit(DMA1_IT_TC7);
        DMA_Cmd(DMA1_Channel7, DISABLE);
        xSemaphoreGiveFromISR(xDMA_Sem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}