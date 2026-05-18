
#include "bsp_tianwen.h"
#include "ch32v30x.h"
#include "ch32v30x_usart.h"
extern void UART_SendBytes(uint8_t *data, uint16_t len);

void Tianwen_Send_Standard_Cmd(uint8_t cmd)
{
    uint8_t send_buf[5];
    
    send_buf[0] = TIANWEN_FRAME_HEAD;    // 0x5A
    send_buf[1] = cmd;
    send_buf[2] = TIANWEN_FIXED_DATA1;   // 0xD5
    send_buf[3] = TIANWEN_FIXED_DATA2;   // 0x5A
    send_buf[4] = TIANWEN_FRAME_TAIL;    // 0xFF
    
    // 补充：调用底层发送将数据推出去
    UART_SendBytes(send_buf, 5);
}
// 记得在文件顶部包含 CH32 的头文件，比如 #include "ch32v30x.h" 或 #include "debug.h"

/**
  * @brief  底层的串口批量发送函数
  * @param  data: 要发送的数据数组指针
  * @param  len: 要发送的字节长度
  */
void UART_SendBytes(uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        USART_SendData(USART3, data[i]);
        // 等待发送完成
        while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
    }
}