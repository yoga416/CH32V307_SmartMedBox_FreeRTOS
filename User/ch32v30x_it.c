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
  /* 进入死循环前，通过串口打印报错信息 */
  /* 注意：如果串口初始化还没完成就崩了，这里可能没输出 */
  printf("\r\n[CRITICAL] !!! HardFault Detected !!!\r\n");
  printf("[System] The processor has crashed due to memory access, stack overflow or invalid instruction.\r\n");

  /* 这种死循环可以防止芯片不断重启，方便你观察串口日志 */
  while (1)
  {
      /* 可以在这里加个 LED 闪烁，肉眼观察是否崩了 */
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

    // ===================================================================
    // 如果后续你的 WIFI(USART) 也用到了 Line 5/7/8/9，直接在下面接着写 if 即可
    // if (EXTI_GetITStatus(EXTI_Line8) != RESET) { ... }
    // ===================================================================

    // 3. 统一收尾：执行 FreeRTOS 的上下文切换
    if (xHigherPriorityTaskWoken == pdTRUE)
    {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}