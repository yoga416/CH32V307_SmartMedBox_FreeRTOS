#include "bsp_tianwen_tesk.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "debug.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "app_task.h"
#include "bsp_tianwen.h"
#include "bsp_tianwen_reg.h"

// 假设你在串口接收中断（USART_IRQHandler）中把收到的数据塞进了这个队列
// 记得在主函数初始化时创建它: Tianwen_Uart_Queue = xQueueCreate(64, sizeof(uint8_t));
extern QueueHandle_t CmdQueue;
extern SemaphoreHandle_t xSem_face_recog; // 人脸识别结果同步信号量
extern SemaphoreHandle_t xSem_face_add; // 人脸识别结果信号量
// --- 天问通信协议指令宏定义 ---
#define CMD_FACE_RECOG     0x02  // 进行人脸识别
#define CMD_MEASURE        0x07  // 进行环境温湿度检测
#define CMD_TEMP_MEASURE   0x0B  // 进行体温测量
#define CMD_HR_SPO2        0x0C  // 进行心率血氧测量
#define CMD_FACE_ADD       0x0D  // 进行人脸添加

/// --- 天问通信数据帧定义 ---
#define TIANWEN_FRAME_HEAD 0x5A
#define TIANWEN_FRAME_TAIL 0xFF

// 天问通信任务
void tianwen_task(void *pvParameters)
{
    (void)pvParameters;
    SystemCommand_t test_cmd;
    Tianwen_Packet_t tianwen_packet;
    uint8_t test_step = 0;
    uint8_t rx_byte = 0;
    uint8_t parse_state = 0;
    uint8_t current_cmd = 0;

    for(;;)
    {
        // 阻塞等待串口队列中的数据。portMAX_DELAY 表示如果没有数据，任务就挂起不占CPU
        if (xQueueReceive(CmdQueue, &rx_byte, 0) == pdPASS)
        {
            // --- 状态机解析 5 字节数据帧：5A 指令 D5 5A FF ---
            switch (parse_state)
            {
                case 0: // 寻找帧头1：0x5A
                    if (rx_byte == 0x5A) parse_state = 1;
                    break;

                case 1: // 提取指令字节
                    current_cmd = rx_byte;
                    parse_state = 2;
                    break;

                case 2: // 校验固定字节：0xD5
                    if (rx_byte == 0xD5) parse_state = 3;
                    else parse_state = 0; // 校验失败，复位状态机，重新找帧头
                    break;

                case 3: // 校验帧头2/固定字节：0x5A
                    if (rx_byte == 0x5A) parse_state = 4;
                    else parse_state = 0;
                    break;

                case 4: // 校验帧尾：0xFF
                    if (rx_byte == 0xFF)
                    {
                        // --------------------------------------------------
                        // 完整数据帧接收成功！开始根据指令执行操作
                        // --------------------------------------------------
                        switch(current_cmd)
                        {
                            case CMD_FACE_RECOG://（人脸识别）- 暂不可用，跳过
                                APP_LOG("[Voice Cmd] Face recognition (skipped, no camera)\n");
                                break;

                            case CMD_MEASURE://（环境温湿度）
                                 APP_LOG("[Voice Cmd] Measure environment temperature & humidity\n");
                                test_cmd = CMD_MEASURE_ENV_TEMP_HUMI;
                                xQueueSend(xSysCmdQueue, &test_cmd, 0);
                                break;

                            case CMD_TEMP_MEASURE://（体温测量）
                                APP_LOG("[Voice Cmd] Measure body temperature\n");
                                test_cmd = CMD_MEASURE_BODY_TEMP;
                                xQueueSend(xSysCmdQueue, &test_cmd, 0);
                                break;

                            case CMD_HR_SPO2://（心率血氧测量）
                                APP_LOG("[Voice Cmd] Measure heart rate & SpO2\n");
                                test_cmd = CMD_MEASURE_HR_SPO2;
                                xQueueSend(xSysCmdQueue, &test_cmd, 0);
                                break;

                            case CMD_FACE_ADD://（人脸添加）
                                APP_LOG("[Voice Cmd] Add face\n");
                                xSemaphoreGive(xSem_face_add);
                                break;

                            default:// 未知指令
                                APP_LOG("[Warning] Unknown command received: 0x%02X\n", current_cmd);
                                break;
                        }
                    }
                    // 无论处理完毕还是帧尾校验错误，都复位状态机准备接收下一帧
                    parse_state = 0; 
                    break;

                default:
                    parse_state = 0;
                    break;
            }
        }

      //接收数据帧并发送给天问模块
      if(xQueueReceive(sendtianwenQueue, &tianwen_packet, pdMS_TO_TICKS(10)) == pdPASS)
      {
         //判断是数据包还是指令包
         if(tianwen_packet.data_len > 0)
         {
             //判断是哪个传感器的数据，进行相应的打包处理
             switch(tianwen_packet.id)
             {
                  case SENSOR_ID_HEART_RATE_SPO2://cmd:09
                  {
                       uint8_t send_buffer[10];
                        send_buffer[0] = TIANWEN_FRAME_HEAD;    // 0x5A
                        send_buffer[1] = 0x09;     // CMD
                        send_buffer[2] = tianwen_packet.data[0];   // 心率高8位
                        send_buffer[3] = tianwen_packet.data[1];   // 心率低8位
                        send_buffer[4] = tianwen_packet.data[2];   // 血氧高8位
                        send_buffer[5] = tianwen_packet.data[3];   // 血氧低8位
                        send_buffer[6] = TIANWEN_FRAME_TAIL;       // 0xFF
                        //调用底层发送函数将数据推出去
                        UART_SendBytes(send_buffer, 7);
                        printf("[Tianwen Task] Sent HR & SpO2 data to Tianwen module\n");
                        break;
                  }
                  case SENSOR_ID_MLX_TEMP_BOTH://cmd:08
                  {
                        uint8_t temp_buffer[10];
                        temp_buffer[0] = TIANWEN_FRAME_HEAD;    // 0x5A
                        temp_buffer[1] = 0x08;     // CMD     
                        temp_buffer[2] = tianwen_packet.data[0];   // 体温高8位
                        temp_buffer[3] = tianwen_packet.data[1];   // 体温低8位
                        temp_buffer[4] = TIANWEN_FRAME_TAIL;       // 0xFF
                        //调用底层发送函数将数据推出去      
                        UART_SendBytes(temp_buffer, 5);
                        printf("[Tianwen Task] Sent MLX temperature data to Tianwen module\n");
                        break;    
                  }  
                  case SENSOR_ID_TEMPERATURE_HUMIDITY://cmd:10
                  {
                        uint8_t env_buffer[10];
                        env_buffer[0] = TIANWEN_FRAME_HEAD;    // 0x5A
                        env_buffer[1] = 0x0A;     // CMD      
                        env_buffer[2] = tianwen_packet.data[0];   // 温度高8位
                        env_buffer[3] = tianwen_packet.data[1];   // 温度低8位
                        env_buffer[4] = tianwen_packet.data[2];   // 湿度高8位
                        env_buffer[5] = tianwen_packet.data[3];   // 湿度低8位
                        env_buffer[6] = TIANWEN_FRAME_TAIL;       // 0xFF
                        //调用底层发送函数将数据推出去
                        UART_SendBytes(env_buffer, 7);
                        printf("[Tianwen Task] Sent SHT40 environment data to Tianwen module\n");
                        break;
                  }
                  default:
                        
                        break;
         }
      }
         else
         {
            // 指令包，直接发指令字节
             //指令字节格式：0x5A CMD 0xD5 0x5A 0xFF
                  uint8_t send_buf[5];
                  send_buf[0] = TIANWEN_FRAME_HEAD;    // 0x5A
                  send_buf[1] = tianwen_packet.id;     // CMD
                  send_buf[2] = TIANWEN_FIXED_DATA1;   // 0xD5
                  send_buf[3] = TIANWEN_FIXED_DATA2;   // 0x5A
                  send_buf[4] = TIANWEN_FRAME_TAIL;    // 0xFF
                  //调用底层发送函数将数据推出去
                  UART_SendBytes(send_buf, 5);
                  if(tianwen_packet.id == CMD_TX_ALARM_TIME_UP)
                  {
                      printf("[Tianwen Task] Sent alarm reminder command to Tianwen module\n");
                  }
                 if(tianwen_packet.id == CMD_TX_BOOT_VOICE)
                  {
                      printf("[Tianwen Task] Sent boot voice command to Tianwen module\n");
                  }
                  if(tianwen_packet.id == CMD_TX_NOT_TIME_TO_EAT)
                  {
                      printf("[Tianwen Task] Sent not time to eat command to Tianwen module\n");
                  }
                  if(tianwen_packet.id == CMD_TX_TIME_TO_EAT)
                  {
                      printf("[Tianwen Task] Sent time to eat command to Tianwen module\n");
                  }
         }
       
    }
    vTaskDelay(pdMS_TO_TICKS(20)); // 避免任务占用过多CPU，适当延时
}
    }

