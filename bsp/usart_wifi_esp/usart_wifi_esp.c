#include "usart_wifi_esp.h"
#include "ch32v30x.h"
#include "ch32v30x_usart.h"
#include "ch32v30x_dma.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "bsp_utils.h" 
#include <string.h>

extern SemaphoreHandle_t xDMA_Sem;
extern QueueHandle_t xUART_Queue;

// 显式声明中断服务函数
void DMA1_Channel7_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

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

/**
 * @brief 接收不定长数据帧 (阻塞式，带超时)
 * 格式：0x5A(头) + Length(1B) + ID(1B) + Data(NB) + CRC(1B) + 0xFF(尾)
 */
usart_wifi_esp_Status_t USART_WIFI_ESP_Receive(Packet_t *packet)
{
    if (packet == NULL || xUART_Queue == NULL) return USART_WIFI_ESP_ERROR;

    uint8_t rx_byte;
    uint8_t state = 0;
    uint8_t payload_idx = 0;
    uint8_t crc_calc_buf[MAX_PAYLOAD_LEN + 2]; // 用于校验的缓冲区 (len + id + payload)
    uint8_t crc_calc_idx = 0;

    // 整个数据包接收的最长等待时间 (USART_WIFI_ESP_TIMEOUT_MS)
    TickType_t xStartTime = xTaskGetTickCount();

    while ((xTaskGetTickCount() - xStartTime) < pdMS_TO_TICKS(USART_WIFI_ESP_TIMEOUT_MS))
    {
        if (xQueueReceive(xUART_Queue, &rx_byte, pdMS_TO_TICKS(10)) == pdPASS)
        {
            switch (state)
            {
                case 0: // 找帧头
                    if (rx_byte == PACKET_HEAD_VAL) state = 1;
                    break;

                case 1: // 获取长度
                    packet->data_len = rx_byte;
                    crc_calc_buf[crc_calc_idx++] = rx_byte; // CRC 包含长度字节
                    if (packet->data_len > MAX_PAYLOAD_LEN) {
                        state = 0; crc_calc_idx = 0; // 长度非法，复位
                    } else {
                        state = 2;
                    }
                    break;

                case 2: // 获取 ID
                    packet->sensor_id = rx_byte;
                    crc_calc_buf[crc_calc_idx++] = rx_byte; // CRC 包含 ID 字节
                    if (packet->data_len == 0) state = 4; // 无负载直接跳过数据接收
                    else {
                        payload_idx = 0;
                        state = 3;
                    }
                    break;

                case 3: // 接收负载数据
                    packet->payload[payload_idx++] = rx_byte;
                    crc_calc_buf[crc_calc_idx++] = rx_byte; // CRC 包含负载字节
                    if (payload_idx >= packet->data_len) state = 4;
                    break;

                case 4: // 校验 CRC
                {
                    uint8_t received_crc = rx_byte;
                    uint8_t expected_crc = bsp_utils_calc_crc8(crc_calc_buf, crc_calc_idx);
                    if (received_crc == expected_crc) {
                        state = 5;
                    } else {
                        state = 0; crc_calc_idx = 0; // 校验失败
                        return USART_WIFI_ESP_ERROR;
                    }
                    break;
                }

                case 5: // 帧尾
                    if (rx_byte == PACKET_TAIL_VAL) {
                        return USART_WIFI_ESP_OK; // 接收成功
                    }
                    state = 0; crc_calc_idx = 0;
                    break;
            }
        }
    }

    return USART_WIFI_ESP_ERROR; // 超时
}

// USART2 中断服务函数
void USART2_IRQHandler(void)
{
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        uint8_t data = USART_ReceiveData(USART2);
        if (xUART_Queue != NULL)
        {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xQueueSendFromISR(xUART_Queue, &data, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
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
