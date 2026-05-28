#include "facial.h"
#include "facial_reg.h"

#include "ch32v30x.h"
#include "ch32v30x_usart.h"
#include "system_config.h"
#include "bsp_utils.h"
// ---------------------------- 环形FIFO缓冲区 ----------------------------
static uint8_t rx_buffer[FR_USART_BUFFER_SIZE];
static volatile uint16_t rx_head = 0;   // 写指针
static volatile uint16_t rx_tail = 0;   // 读指针

// 向FIFO写入一个字节（由中断调用）
static void fifo_put(uint8_t data) {
    uint16_t next = (rx_head + 1) % FR_USART_BUFFER_SIZE;
    if (next != rx_tail) {
        rx_buffer[rx_head] = data;
        rx_head = next;
    }
    // 如果FIFO满，丢弃数据（可根据需要增加溢出计数）
}

// 从FIFO读取一个字节，成功返回1，失败返回0
static uint8_t fifo_get(uint8_t *data) {
    if (rx_head == rx_tail) return 0;
    *data = rx_buffer[rx_tail];
    rx_tail = (rx_tail + 1) % FR_USART_BUFFER_SIZE;
    return 1;
}

// 清空FIFO
static void fifo_clear(void) {
    rx_head = 0;
    rx_tail = 0;
}
// ---------------------------- USART5初始化（开启接收中断） ----------------------------
void FR_USART_Init(void) {
    // 1. 时钟使能
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);

    // 2. TX引脚配置（PC12 复用推挽）
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = FR_USART_TX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(FR_USART_TX_PORT, &GPIO_InitStructure);

    // 3. RX引脚配置（PD2 浮空输入）
    GPIO_InitStructure.GPIO_Pin = FR_USART_RX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(FR_USART_RX_PORT, &GPIO_InitStructure);

    // 4. USART参数配置
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = FR_USART_BAUDRATE;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(UART5, &USART_InitStructure);

    // 5. 使能接收中断
    USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);

    // 6. 配置NVIC（UART5中断通道）
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // 7. 使能USART
    USART_Cmd(UART5, ENABLE);

    fifo_clear();
}


// ---------------------------- 发送数据包 ----------------------------
void FR_SendPacket(FR_packet_t *packet) {
    if (packet == NULL || packet->data_len > FR_MAX_PAYLOAD_LEN) return;

    // 构建CRC校验数据段：帧头 + sensor_id + data_len + payload
    uint8_t crc_buf[3 + FR_MAX_PAYLOAD_LEN];
    crc_buf[0] = FR_PACKET_HEAD_VAL;
    crc_buf[1] = packet->sensor_id;
    crc_buf[2] = packet->data_len;
    memcpy(&crc_buf[3], packet->payload, packet->data_len);
    uint8_t crc = bsp_utils_calc_crc8(crc_buf, 3 + packet->data_len);

    // 发送帧头
    USART_SendData(UART5, FR_PACKET_HEAD_VAL);
    while (USART_GetFlagStatus(UART5, USART_FLAG_TXE) == RESET);

    // 发送传感器ID
    USART_SendData(UART5, packet->sensor_id);
    while (USART_GetFlagStatus(UART5, USART_FLAG_TXE) == RESET);

    // 发送数据长度
    USART_SendData(UART5, packet->data_len);
    while (USART_GetFlagStatus(UART5, USART_FLAG_TXE) == RESET);

    // 发送净荷
    for (uint8_t i = 0; i < packet->data_len; i++) {
        USART_SendData(UART5, packet->payload[i]);
        while (USART_GetFlagStatus(UART5, USART_FLAG_TXE) == RESET);
    }

    // 发送CRC
    USART_SendData(UART5, crc);
    while (USART_GetFlagStatus(UART5, USART_FLAG_TXE) == RESET);

    // 发送帧尾
    USART_SendData(UART5, FR_PACKET_TAIL_VAL);
    while (USART_GetFlagStatus(UART5, USART_FLAG_TXE) == RESET);

    // 等待发送完成
    while (USART_GetFlagStatus(UART5, USART_FLAG_TC) == RESET);
}

// ---------------------------- USART5中断服务函数 ----------------------------
void UART5_IRQHandler(void) {
    if (USART_GetITStatus(UART5, USART_IT_RXNE) != RESET) {
        uint8_t rx_data = USART_ReceiveData(UART5);
        fifo_put(rx_data);
        USART_ClearITPendingBit(UART5, USART_IT_RXNE);
    }
}

// ---------------------------- 从FIFO中解析一个完整数据包（非阻塞） ----------------------------
uint8_t FR_GetPacket(FR_packet_t *packet) {
    static uint8_t state = 0;              // 状态机状态
    static uint8_t sensor_id, data_len;
    static uint8_t payload[FR_MAX_PAYLOAD_LEN];
    static uint8_t payload_index;
    static uint8_t recv_crc, calc_crc_val;
    uint8_t byte;

    while (fifo_get(&byte)) {
        switch (state) {
            case 0: // 等待帧头
                if (byte == FR_PACKET_HEAD_VAL) {
                    state = 1;
                }
                break;
            case 1: // 传感器ID
                sensor_id = byte;
                state = 2;
                break;
            case 2: // 数据长度
                if (byte > FR_MAX_PAYLOAD_LEN) {
                    state = 0; // 非法长度，丢弃
                } else {
                    data_len = byte;
                    payload_index = 0;
                    state = 3;
                }
                break;
            case 3: // 接收payload
                payload[payload_index++] = byte;
                if (payload_index == data_len) {
                    state = 4;
                }
                break;
            case 4: // 接收CRC
                recv_crc = byte;
                // 计算校验值
                {
                    uint8_t crc_buf[3 + FR_MAX_PAYLOAD_LEN];
                    crc_buf[0] = FR_PACKET_HEAD_VAL;
                    crc_buf[1] = sensor_id;
                    crc_buf[2] = data_len;
                    memcpy(&crc_buf[3], payload, data_len);
                    calc_crc_val = bsp_utils_calc_crc8(crc_buf, 3 + data_len);
                }
                state = 5;
                break;
            case 5: // 接收帧尾
                if (byte == FR_PACKET_TAIL_VAL && recv_crc == calc_crc_val) {
                    // 校验成功，拷贝到输出结构体
                    packet->sensor_id = sensor_id;
                    packet->data_len = data_len;
                    memcpy(packet->payload, payload, data_len);
                    state = 0;
                    return 1; // 成功得到一个完整包
                    printf("[FR] Received valid packet: ID=0x%02X, Len=%d\n", sensor_id, data_len);
                } else {
                    state = 0; // 校验失败，丢弃
                    printf("[FR] Received invalid packet: ID=0x%02X, Len=%d\n", sensor_id, data_len);
                }
                break;
            default:
                state = 0;
                break;
        }
    }
    return 0; // 未收到完整包
}