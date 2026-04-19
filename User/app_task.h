#ifndef __APP_TASK_H
#define __APP_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "Middle_ring_buffer.h"
/* 任务优先级定义 */
#define USART_TASK_PRIO     4
#define SENSOR_TASK_PRIO    4
#define APP_TASK_PRIO       4

/* 堆栈大小转换宏 */
#define STACK_BYTES_TO_WORDS(bytes) ((uint16_t)((bytes) / sizeof(StackType_t)))

/* 任务堆栈大小定义 */
#define USART_TASK_STK_SIZE  STACK_BYTES_TO_WORDS(1024) // 1KB
#define SENSOR_TASK_STK_SIZE STACK_BYTES_TO_WORDS(2048) // 2KB (因为里面有printf和队列)
#define APP_TASK_STK_SIZE    STACK_BYTES_TO_WORDS(1024) // 1KB

/* 传感器模式配置 */
#define SENSOR_MODE_NOTSENSOR 0
#define SENSOR_MODE_MAX30102 1
#define SENSOR_MODE_SHT40    2
#define SENSOR_MODE_MLX90614 3
#define SENSOR_MODE_BATTERY_CHECK 4
#define SENSOR_MODE_SIM800L 5
#define SENSOR_RUN_MODE      SENSOR_MODE_MAX30102




/* 全局变量声明 */
extern TaskHandle_t bspSensorTask_Handler;
extern TaskHandle_t appTask_Handler;
extern TaskHandle_t usartTask_Handler;
extern RingBuffer_t g_ring_buffer;
extern QueueHandle_t xUART_Queue;
extern SemaphoreHandle_t xDMA_Sem;
extern SemaphoreHandle_t xBufferDataSemaphore;

/* 全局函数声明 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName);
void vApplicationMallocFailedHook(void);

#endif /* __APP_TASK_H */