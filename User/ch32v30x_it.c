/********************************** (C) COPYRIGHT *******************************
* File Name          : ch32v30x_it.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2024/03/05
* Description        : Main Interrupt Service Routines.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "ch32v30x_it.h"
#include "system.h"
#include "semphr.h"
#include "../bsp_rtc/bsp_rtc.h"
#include "../bsp/bsp_tianwen/bsp_tianwen_reg.h"
#include "app_task.h"

void NMI_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void HardFault_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));


extern SemaphoreHandle_t xDMA_Sem; // DMA发送完成信号量
extern EventGroupHandle_t xEventGroup; // 事件组句柄
/*********************************************************************
 * @fn      NMI_Handler
 *
 * @brief   This function handles NMI exception.
 *
 * @return  none
 */
void NMI_Handler(void)
{
  while (1)
  {
  }
}

/*********************************************************************
 * @fn      HardFault_Handler
 *
 * @brief   This function handles Hard Fault exception.
 *
 * @return  none
 */
// void HardFault_Handler(void)
// { 
//   __disable_irq();
//   while (1)
//   {
    
//   }  
// }
void HardFault_Handler(void)
{
    /* 1. 定义变量存储关键寄存器 */
    uint32_t mepc, mcause, mtval;

    /* 2. 使用 RISC-V 内联汇编读取 CSR 寄存器 */
    __asm volatile("csrr %0, mepc"   : "=r"(mepc));   // 获取崩溃时的程序指针 (PC)
    __asm volatile("csrr %0, mcause" : "=r"(mcause)); // 获取导致异常的原因
    __asm volatile("csrr %0, mtval"  : "=r"(mtval));  // 获取导致异常的内存地址或指令

    /* 3. 打印核心崩溃现场 */
    printf("\r\n========================================\r\n");
    printf("[CRITICAL] !!! HardFault Detected !!!\r\n");
    printf("========================================\r\n");
    printf("MEPC   (Crash PC)   : 0x%08lX\r\n", mepc);
    printf("MCAUSE (Reason)     : 0x%08lX\r\n", mcause);
    printf("MTVAL  (Fault Addr) : 0x%08lX\r\n", mtval);
    printf("========================================\r\n");

    /* 4. 尝试获取并打印当前正在运行的 FreeRTOS 任务名 */
    /* 注意：如果由于内存越界导致 FreeRTOS 内核数据结构损坏，
       调用 pcTaskGetName 可能会引发二次崩溃，导致打印不全。
       如果发现卡死且没印出这句话，可以把这行注释掉。 */
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) 
    {
        printf("[Debug] Current Task: %s\r\n", pcTaskGetName(NULL));
    }
    
    printf("System halted. Please check the logs above.\r\n");

    /* 5. 死循环，防止芯片不断重启 */
    while (1)
    {
        /* 如果有 LED，可以加一段极其简单的翻转代码，比如：
           GPIOA->out ^= (1<<0); // 伪代码，根据你的引脚修改
           for(volatile int i=0; i<1000000; i++); // 粗延时
        */
    }
}



void  TIM2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM2_IRQHandler(void)
{
  if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
  {
    // 在中断服务函数(ISR)中设置标志位
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
  }
}

// void EXTI9_5_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
// void EXTI9_5_IRQHandler(void)
// {
//   if (EXTI_GetITStatus(EXTI_Line6) != RESET)
//   {
//     BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//     EXTI_ClearITPendingBit(EXTI_Line6);
//     if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
//       xEventGroupSetBitsFromISR(xEventGroup, EVENT_SENSOR_DATA_READY, &xHigherPriorityTaskWoken);
//       portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//     }
//   }
// }



// 引入外部的 RTOS 句柄
extern EventGroupHandle_t xEventGroup;
extern SemaphoreHandle_t xSem_MAX30102_Exti;

// 定义事件位（确保与你 app_task 中的定义一致）
#define EVENT_SENSOR_DATA_READY (1 << 0) 


/* ================= 整合后的 EXTI9_5 中断服务函数 ================= */
void EXTI9_5_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI9_5_IRQHandler(void)
{
    // 用于记录是否有高优先级任务被唤醒，统一在最后执行切换
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    BaseType_t xTaskWoken = pdFALSE;

    // 判断是否是 EXTI_Line6 (MAX30102 引脚) 触发的中断
    if (EXTI_GetITStatus(EXTI_Line6) != RESET)
    {
        // 1. 尽早清除中断标志位
        EXTI_ClearITPendingBit(EXTI_Line6);

        // 2. 安全检查：确保 FreeRTOS 调度器已经处于运行状态
        if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) 
        {
            // --- 机制 A：设置事件标志组 ---
            if (xEventGroup != NULL) {
                xEventGroupSetBitsFromISR(xEventGroup, EVENT_SENSOR_DATA_READY, &xTaskWoken);
                // 如果机制A唤醒了更高优先级任务，登记下来
                if (xTaskWoken == pdTRUE) xHigherPriorityTaskWoken = pdTRUE; 
            }

            // --- 机制 B：释放二值信号量 ---
            if (xSem_MAX30102_Exti != NULL) {
                xTaskWoken = pdFALSE; // 复位局部变量
                xSemaphoreGiveFromISR(xSem_MAX30102_Exti, &xTaskWoken);
                // 如果机制B唤醒了更高优先级任务，登记下来
                if (xTaskWoken == pdTRUE) xHigherPriorityTaskWoken = pdTRUE;
            }
        }
    }

    if (xHigherPriorityTaskWoken == pdTRUE)
    {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

//RTC 秒中断服务函数
 void RTC_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

void RTC_IRQHandler(void) {
    //闹钟中断
    if (RTC_GetITStatus(RTC_IT_ALR) != RESET) {
        
        // 1. 执行你的提醒逻辑
        //吃药时间到了，执行提醒操作，比如点亮LED，或者发送消息到app_task.c让它去播报语音
        APP_LOG("Alarm Triggered! Time to take your medicine!\r\n");
        //发送语音播报命令到 app_task.c
        Tianwen_Packet_t tianwen_packet;
        tianwen_packet.id = CMD_TX_ALARM_TIME_UP; // 这里可以定义一个新的指令，比如 CMD_MED_REMINDER
        tianwen_packet.data_len = 0; // 没有额外数据(命令帧)
        xQueueSendFromISR(sendtianwenQueue, &tianwen_packet, NULL);
        RTC_TimeTypeDef current_time;
        BSP_RTC_GetDateTime(&current_time); // 获取当前时间
        //显示在界面上，或者发送给app_task.c中的时间更新队列,显示在界面上
         APP_LOG("Current Time: %04d-%02d-%02d %02d:%02d:%02d\r\n", 
                current_time.year, current_time.month, current_time.day, 
                current_time.hour, current_time.min, current_time.sec);
         // 2. 更新下一个闹钟时间，实现循环提醒
        BSP_RTC_UpdateNextAlarm();

        // 3. 清除标志位
        RTC_ClearITPendingBit(RTC_IT_ALR);
        RTC_WaitForLastTask();
    }
    
    // 秒中断
    if (RTC_GetITStatus(RTC_IT_SEC) != RESET) {
        RTC_ClearITPendingBit(RTC_IT_SEC);
        //每5秒更新一次界面上的时间显示，或者发送给app_task.c中的时间更新队列,显示在界面上
        RTC_TimeTypeDef current_time;
        BSP_RTC_GetDateTime(&current_time); // 获取当前时间
        APP_LOG("Current Time: %04d-%02d-%02d %02d:%02d:%02d\r\n", 
                current_time.year, current_time.month, current_time.day, 
                current_time.hour, current_time.min, current_time.sec);
        RTC_WaitForLastTask();
    }
}

//天问通信串口中断服务函数
void USART3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
// 声明那个队列，确保它已经在主函数里被 xQueueCreate 初始化过了
extern QueueHandle_t CmdQueue; // 天问通信指令队列

// USART3 中断服务函数
void USART3_IRQHandler(void)
{
    uint8_t rx_data;
    // 用于标记是否需要进行任务切换
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // 判断是否是接收非空中断 (RXNE)
    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        // 1. 读取接收到的字节（读取 DR 寄存器会自动清除 RXNE 标志位）
        rx_data = (uint8_t)USART_ReceiveData(USART3);
        
        // 2. 将数据发送到 FreeRTOS 队列中
        // 注意：中断里绝对不能用 xQueueSend，必须用 xQueueSendFromISR
        if(CmdQueue != NULL)
        {
            xQueueSendFromISR(CmdQueue, &rx_data, &xHigherPriorityTaskWoken);
        }
        
        // 保险起见，手动清一下中断标志位
        USART_ClearITPendingBit(USART3, USART_IT_RXNE);
    }
    
    // 3. FreeRTOS 关键一步：如果刚刚入队的数据唤醒了一个高优先级任务（比如我们一直阻塞等待数据的天问解析任务），
    // 那么退出中断后立刻进行任务调度，保证实时性。
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

}

/* xTask: 发生溢出的任务句柄 */
/* pcTaskName: 发生溢出的任务名称 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    /* 当程序运行到这里，说明堆栈已经崩了 */
    
    // 1. 可以在这里打一个断点，方便调试
    // 2. 串口打印是谁崩了
    APP_LOG("Stack Overflow in task: %s\r\n", pcTaskName);
    
    // 3. 进入死循环，防止错误扩大
    while(1);
}
    
