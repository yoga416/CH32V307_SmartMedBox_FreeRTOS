#include "facial.h"
#include "bsp_utils.h" // 包含 bsp_utils_calc_crc8 等工具函数
#include <string.h>
#include <stdio.h>

// ---------------- 内部变量与 RTOS 资源 ----------------
static QueueHandle_t      xFR_RxQueue = NULL;      // 接收消息队列
static SemaphoreHandle_t  xFR_TxDmaMutex = NULL;   // 发送互斥锁（确保多任务调用发送安全）
static SemaphoreHandle_t  xFR_TxDmaSem = NULL;     // DMA 发送完成同步信号量

static uint8_t FR_TxBuffer[FR_MAX_PAYLOAD_LEN + 8]; // DMA发送缓存

// ---------------- 硬件初始化 ----------------
void FR_Init(void) {
    GPIO_InitTypeDef  GPIO_InitStructure = {0};
    USART_InitTypeDef USART_InitStructure = {0};
    DMA_InitTypeDef   DMA_InitStructure = {0};
    NVIC_InitTypeDef  NVIC_InitStructure = {0};

    // 1. 创建 FreeRTOS 资源
    xFR_RxQueue = xQueueCreate(FR_RX_QUEUE_LENGTH, sizeof(uint8_t));
    xFR_TxDmaMutex = xSemaphoreCreateMutex();
    xFR_TxDmaSem = xSemaphoreCreateBinary();

    // 2. 使能时钟
    RCC_APB2PeriphClockCmd(FR_USART_TX_RCC | FR_USART_RX_RCC, ENABLE);
    RCC_APB1PeriphClockCmd(FR_USART_RCC, ENABLE);
    RCC_AHBPeriphClockCmd(FR_TX_DMA_RCC, ENABLE);

    // 3. 配置 GPIO
    GPIO_InitStructure.GPIO_Pin = FR_USART_TX_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(FR_USART_TX_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = FR_USART_RX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(FR_USART_RX_PORT, &GPIO_InitStructure);

    // 4. 配置 USART
    USART_InitStructure.USART_BaudRate = FR_USART_BAUDRATE;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(FR_USART, &USART_InitStructure);

    // 5. 配置 TX DMA
    DMA_DeInit(FR_TX_DMA_CHANNEL);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&FR_USART->DATAR);
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)FR_TxBuffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = 0;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(FR_TX_DMA_CHANNEL, &DMA_InitStructure);
    
    // 使能 DMA 发送完成中断
    DMA_ITConfig(FR_TX_DMA_CHANNEL, DMA_IT_TC, ENABLE);
    
    // 6. 配置中断优先级
    // USART RX 中断
    NVIC_InitStructure.NVIC_IRQChannel = FR_USART_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5; // 根据FreeRTOSConfig.h配置
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // DMA TX 中断
    NVIC_InitStructure.NVIC_IRQChannel = FR_TX_DMA_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // 7. 使能 USART 和 接收中断
    USART_ITConfig(FR_USART, USART_IT_RXNE, ENABLE);
    USART_DMACmd(FR_USART, USART_DMAReq_Tx, ENABLE);
    USART_Cmd(FR_USART, ENABLE);
}

// ---------------- 异步/DMA发送包 ----------------
fr_status_t FR_SendPacket(FR_packet_t *packet) {
    if (packet == NULL || packet->data_len > FR_MAX_PAYLOAD_LEN) {
        return FR_ERR_PARAM;
    }

    // 获取发送互斥锁，防止多任务争抢串口发送
    if (xSemaphoreTake(xFR_TxDmaMutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return FR_ERR_BUSY;
    }

    uint16_t len = 0;
    FR_TxBuffer[len++] = FR_PACKET_HEAD_VAL;
    FR_TxBuffer[len++] = packet->sensor_id;
    FR_TxBuffer[len++] = packet->data_len;
    
    // 拷贝 Payload
    memcpy(&FR_TxBuffer[len], packet->payload, packet->data_len);
    len += packet->data_len;
    
    // 计算 CRC (计算包头到Payload的数据)
    uint8_t crc = bsp_utils_calc_crc8(FR_TxBuffer, len);
    FR_TxBuffer[len++] = crc;
    FR_TxBuffer[len++] = FR_PACKET_TAIL_VAL;

    // 启动 DMA 传输
    DMA_Cmd(FR_TX_DMA_CHANNEL, DISABLE);
    FR_TX_DMA_CHANNEL->CNTR = len; // 设置传输长度
    DMA_Cmd(FR_TX_DMA_CHANNEL, ENABLE);

    // 等待 DMA 发送完成中断释放信号量
    fr_status_t status = FR_OK;
    if (xSemaphoreTake(xFR_TxDmaSem, pdMS_TO_TICKS(500)) != pdTRUE) {
        // 超时处理：关闭DMA
        DMA_Cmd(FR_TX_DMA_CHANNEL, DISABLE);
        status = FR_ERR_TIMEOUT;
    }

    xSemaphoreGive(xFR_TxDmaMutex); // 释放互斥锁
    return status;
}

// ---------------- 阻塞读取并解析完整数据包 ----------------
fr_status_t FR_ReceivePacket_Block(FR_packet_t *packet, uint32_t timeout_ms) {
    static uint8_t state = 0;
    static uint8_t sensor_id, data_len, payload_index;
    static uint8_t payload[FR_MAX_PAYLOAD_LEN];
    static uint8_t recv_crc;
    
    uint8_t byte;
    uint32_t start_ticks = xTaskGetTickCount();
    uint32_t current_ticks;

    // 状态机解析（带有超时退出机制）
    while (1) {
        current_ticks = xTaskGetTickCount();
        uint32_t elapsed = (current_ticks - start_ticks) * portTICK_PERIOD_MS;
        
        if (elapsed >= timeout_ms) {
            return FR_ERR_TIMEOUT;
        }

        // 从 FreeRTOS 队列中读取 1 字节（带有剩余等待时间）
        if (xQueueReceive(xFR_RxQueue, &byte, pdMS_TO_TICKS(timeout_ms - elapsed)) == pdTRUE) {
            switch (state) {
                case 0:
                    if (byte == FR_PACKET_HEAD_VAL) state = 1;
                    break;
                case 1:
                    sensor_id = byte;
                    state = 2;
                    break;
                case 2:
                    if (byte > FR_MAX_PAYLOAD_LEN) {
                        state = 0; // 长度错误，丢弃
                    } else {
                        data_len = byte;
                        payload_index = 0;
                        state = (data_len == 0) ? 4 : 3;
                    }
                    break;
                case 3:
                    payload[payload_index++] = byte;
                    if (payload_index >= data_len) state = 4;
                    break;
                case 4:
                    recv_crc = byte;
                    state = 5;
                    break;
                case 5:
                    if (byte == FR_PACKET_TAIL_VAL) {
                        // 校验 CRC
                        uint8_t crc_buf[3 + FR_MAX_PAYLOAD_LEN];
                        crc_buf[0] = FR_PACKET_HEAD_VAL;
                        crc_buf[1] = sensor_id;
                        crc_buf[2] = data_len;
                        memcpy(&crc_buf[3], payload, data_len);
                        uint8_t calc_crc = bsp_utils_calc_crc8(crc_buf,  3 + data_len);
                        if (recv_crc == calc_crc) {
                            packet->sensor_id = sensor_id;
                            packet->data_len = data_len;
                            memcpy(packet->payload, payload, data_len);
                            state = 0;
                            return FR_OK; // 成功解析出一包
                        }
                    }
                    state = 0; // 帧尾不匹配或 CRC 错误，复位状态机
                    break;
                default:
                    state = 0;
                    break;
            }
        }
    }
}

// ---------------- 中断服务函数 ----------------
// UART5 接收中断
void FR_USART_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void FR_USART_IRQHandler(void) {
    if (USART_GetITStatus(FR_USART, USART_IT_RXNE) != RESET) {
        uint8_t rx_data = USART_ReceiveData(FR_USART);
        
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        // 将接收到的字节送入队列
        xQueueSendFromISR(xFR_RxQueue, &rx_data, &xHigherPriorityTaskWoken);
        
        // 如果因为发送消息唤醒了更高优先级的任务，进行上下文切换
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

// DMA 发送完成中断
void FR_TX_DMA_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void FR_TX_DMA_IRQHandler(void) {
    if (DMA_GetITStatus(FR_TX_DMA_FLAG_TC) != RESET) {
        DMA_ClearITPendingBit(FR_TX_DMA_FLAG_TC);
        DMA_Cmd(FR_TX_DMA_CHANNEL, DISABLE);
        
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        // 释放二值信号量，通知发送任务 DMA 已经完成
        xSemaphoreGiveFromISR(xFR_TxDmaSem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}