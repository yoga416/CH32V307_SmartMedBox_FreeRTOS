/*
 * File: usart_wifi_esp.c
 * Brief: ESP8266 UART/DMA communication implementation.
 */

#include "usart_wifi_esp.h"
#include "ch32v30x.h"
#include "ch32v30x_usart.h"
#include "ch32v30x_dma.h"
#include "FreeRTOS.h"
#include "semphr.h"

extern SemaphoreHandle_t xDMA_Sem;
void DMA1_Channel7_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

usart_wifi_esp_Status_t UART2_DMA_Start_Send(uint8_t *pbuf, uint16_t len)
{
      if (pbuf == 0 || len == 0) {
            return USART_WIFI_ESP_ERROR;
      }

      DMA_Cmd(DMA1_Channel7, DISABLE);
      DMA1_Channel7->MADDR = (uint32_t)pbuf;
      DMA_SetCurrDataCounter(DMA1_Channel7, len);
      USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
      DMA_Cmd(DMA1_Channel7, ENABLE);

      return USART_WIFI_ESP_OK;
}

usart_wifi_esp_Status_t USART_WIFI_ESP_Send(const Packet_t *packet)
{
      if (packet == 0) {
            return USART_WIFI_ESP_ERROR;
      }
      uint16_t send_total_len = sizeof(Packet_t);   
       UART2_DMA_Start_Send((uint8_t *)packet, send_total_len);

     // 必须在这里等待，确保 packet 内存被 DMA 读完之前不被释放或修改
      if(xSemaphoreTake(xDMA_Sem, pdMS_TO_TICKS(100)) == pdTRUE) {
          return USART_WIFI_ESP_OK;
      } else {
          return USART_WIFI_ESP_ERROR;
      }

}

usart_wifi_esp_Status_t USART_WIFI_ESP_Receive(Packet_t *packet)
{
      (void)packet;
      return USART_WIFI_ESP_ERROR;
}


/*********************************************************************
 * @fn      DMA1_Channel7_IRQHandler
 *
 * @brief   This function handles DMA1 Channel7 interrupt.
 *
 * @return  none
 */
void DMA1_Channel7_IRQHandler(void)
{
      if (DMA_GetITStatus(DMA1_IT_TC7) != RESET) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            DMA_ClearITPendingBit(DMA1_IT_TC7);
            //禁用DMA通道，确保不会有新的传输开始
            DMA_Cmd(DMA1_Channel7, DISABLE);
            //释放信号量通知USART任务DMA传输完成
            xSemaphoreGiveFromISR(xDMA_Sem, &xHigherPriorityTaskWoken);

            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
      }
}
