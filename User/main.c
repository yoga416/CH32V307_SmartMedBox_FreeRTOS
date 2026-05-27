/**
* File Name          : main.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2021/06/06
* Description        : Main program body.
*/

#include "debug.h"
#include "FreeRTOS.h"
#include "task.h"
#include "system_config.h"
#include "system.h"
#include "sht40.h"
#include "semphr.h"
#include "queue.h"
#include "../bsp/usart_wifi_esp/usart_wifi_esp.h"
#include "string.h"
#include "Middle_ring_buffer.h"
#include <stdio.h>

#include "app_task.h"
#include "ch32v30x_iwdg.h"


/* watchdog task handle (defined in app_task.c) */
extern TaskHandle_t watchdogTask_Handler;
extern TaskHandle_t tianwenTask_Handler;
extern TaskHandle_t lcdTask_Handler; // 如果有 LCD 任务的话
// Include the header for the MAX30102 sensor

/* 任务函数声明 */
extern void bsp_sensor_task(void *pvParameters);
extern void usart_task(void *pvParameters);
extern void app_task(void *pvParameters);
extern void tianwen_task(void *pvParameters);
extern void lcd_task(void *pvParameters); // 如果有 LCD 任务的话
extern void sensor_lcd_task(void *pvParameters); // 传感器数据 LCD 显示任务

#include "information.h"

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    SystemCoreClockUpdate();
    Delay_Init();
    USART_Printf_Init(115200);


     /* 创建互斥锁和队列 */
    // cm_backtrace_init("CmBackTrace", HARDWARE_VERSION, SOFTWARE_VERSION);
    printf("\r\n==================start======================\r\n");
    uint8_t ret = 0;
    ret = g_peripheral_init();
    if (ret != SYSTEM_INIT_OK) {
        while (1) {} // 初始化失败，停在这里
    } 
    
    // 创建任务并启动调度器
    if (xTaskCreate((TaskFunction_t )app_task,
                        (const char*    )"app_task",
                        (uint16_t       )APP_TASK_STK_SIZE,
                        (void*          )NULL,
                        (UBaseType_t    )APP_TASK_PRIO,
                        (TaskHandle_t*  )&appTask_Handler) != pdPASS)
    {
        printf("[Error] Create app_task failed\n");
        while (1) {}
    }
    //传感器任务
    if (xTaskCreate((TaskFunction_t )bsp_sensor_task,
                        (const char*    )"bsp_sensor_task",
                        (uint16_t       )SENSOR_TASK_STK_SIZE,
                        (void*          )NULL,
                        (UBaseType_t    )SENSOR_TASK_PRIO,
                        (TaskHandle_t*  )&bspSensorTask_Handler) != pdPASS)
    {
        printf("[Error] Create bsp_sensor_task failed\n");
        while (1) {}
    }
    // USART 任务（处理与 ESP8266的通信）
    if (xTaskCreate((TaskFunction_t )usart_task,
                        (const char*    )"usart_task",
                        (uint16_t       )USART_TASK_STK_SIZE,
                        (void*          )NULL,
                        (UBaseType_t    )USART_TASK_PRIO,
                        (TaskHandle_t*  )&usartTask_Handler) != pdPASS)
    {
        printf("[Error] Create usart_task failed\n");
        while (1) {}
    }
    //创建天问通信任务
    if (xTaskCreate((TaskFunction_t )tianwen_task,
                        (const char*    )"tianwen_task",
                        (uint16_t       )TIANWEN_TASK_STK_SIZE,
                        (void*          )NULL,
                        (UBaseType_t    )TIANWEN_TASK_PRIO,
                        (TaskHandle_t*  )&tianwenTask_Handler) != pdPASS)
    {
        printf("[Error] Create tianwen_task failed\n");
        while (1) {}
    }
    /* 创建周期喂狗任务（在 app_task.c 中实现） */
    if (xTaskCreate((TaskFunction_t )watchdog_task,
                        (const char*    )"watchdog_task",
                        (uint16_t       )WATCHDOG_TASK_STK_SIZE,
                        (void*          )NULL,
                        (UBaseType_t    )WATCHDOG_TASK_PRIO,
                        (TaskHandle_t*  )&watchdogTask_Handler) != pdPASS)
    {
        printf("[Error] Create watchdog_task failed\n");
        while (1) {}
    }
    //创建ui任务
    if(xTaskCreate((TaskFunction_t )lcd_task,
                        (const char*    )"lcd_task",
                        (uint16_t       )LCD_TASK_STK_SIZE,
                        (void*          )NULL,
                        (UBaseType_t    )LCD_TASK,
                        (TaskHandle_t*  )&lcdTask_Handler) != pdPASS)
    {
        printf("[Error] Create lcd_task failed\n");
        while (1) {}
    }

    //创建传感器lcd显示任务
    if(xTaskCreate((TaskFunction_t )sensor_lcd_task,
                        (const char*    )"sensor_lcd_task",
                        (uint16_t       )SENSOR_TASK_STK_SIZE,
                        (void*          )NULL,
                        (UBaseType_t    )SENSOR_LCD_TASK,
                        NULL) != pdPASS)
    {
        printf("[Error] Create sensor_lcd_task failed\n");
        while (1) {}
    }
    
    printf("[DEBUG] All tasks created, starting scheduler...\r\n");
    vTaskStartScheduler();
    /* 如果程序运行到这里，说明内存绝对爆了，调度器没起来 */
    printf("[FATAL] Scheduler failed! Remaining Heap: %d bytes\r\n", xPortGetFreeHeapSize());
    while (1)
    {
        printf("shouldn't run at here!!\n");
    }
}
