#ifndef __APP_TASK_H
#define __APP_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "event_groups.h"
#include "Middle_ring_buffer.h"
#include "information.h"
/* 任务优先级定义 */
#define USART_TASK_PRIO     4
#define SENSOR_TASK_PRIO    2
#define APP_TASK_PRIO       3
#define TIANWEN_TASK_PRIO    4
#define LCD_TASK            3 
#define WATCHDOG_TASK_PRIO   4
#define BSP_SHT40_TASK_PRIO  3
#define BSP_MLX90614_TASK_PRIO 3
#define BSP_MAX30102_TASK_PRIO 3
#define SENSOR_LCD_TASK    1
/* 堆栈大小转换宏 */
#define STACK_BYTES_TO_WORDS(bytes) ((uint16_t)((bytes) / sizeof(StackType_t)))

/* 任务堆栈大小定义 */
#define USART_TASK_STK_SIZE  STACK_BYTES_TO_WORDS(1024) // 1KB
#define APP_TASK_STK_SIZE    STACK_BYTES_TO_WORDS(2048) // 2KB
#define TIANWEN_TASK_STK_SIZE STACK_BYTES_TO_WORDS(2048) // 2KB (天问通信任务)
#define LCD_TASK_STK_SIZE   STACK_BYTES_TO_WORDS(4096) // 8KB (增加LVGL栈)
#define WATCHDOG_TASK_STK_SIZE STACK_BYTES_TO_WORDS(512) // 512B (喂狗任务，比较简单)
#define SENSOR_TASK_STK_SIZE STACK_BYTES_TO_WORDS(4096) // 8KB (因为里面有printf和队列)


#define BSP_SHT40_TASK_STK_SIZE  STACK_BYTES_TO_WORDS(2048)  // 2048B (SHT40 Handler 专用)
#define BSP_MLX90614_TASK_STK_SIZE  STACK_BYTES_TO_WORDS(2048)  // 2048B (MLX90614 Handler 专用)
#define BSP_MAX30102_TASK_STK_SIZE  STACK_BYTES_TO_WORDS(2048)  // 2048B (MAX30102 Handler 专用)

// 在 app_task.h 中添加
typedef enum {
    CMD_MEASURE_HR_SPO2,       // 测量血氧心率 (MAX30102)
    CMD_MEASURE_BODY_TEMP,     // 测量体温 (MLX90614)
    CMD_MEASURE_ENV_TEMP_HUMI, // 测量环境温湿度 (SHT40)
    CMD_SEND_SMS_ALERT         // 发送短信报警 (SIM800L)
} SystemCommand_t;


extern QueueHandle_t xSysCmdQueue; // 声明全局系统命令队列
extern QueueHandle_t sendtianwenQueue; // 声明全局天问通信命令队列
extern RingBuffer_t g_ring_buffer; // 声明全局环形缓冲区实例
extern QueueHandle_t sensor_data_lcd_queue; // 声明全局传感器数据 LCD 显示队列
extern SemaphoreHandle_t xGuiMutex; // LVGL 互斥锁，保护所有 GUI 操作

/* ========== 看门狗事件组 ========== */
/* 每个任务对应一个 bit，任务执行一次即置位自己的 bit */
#define WDOG_BIT_APP_TASK       (1 << 0)   // app_task
#define WDOG_BIT_SENSOR_TASK    (1 << 1)   // bsp_sensor_task
#define WDOG_BIT_USART_TASK     (1 << 2)   // usart_task
#define WDOG_BIT_TIANWEN_TASK   (1 << 3)   // tianwen_task
#define WDOG_BIT_LCD_TASK       (1 << 4)   // lcd_task
#define WDOG_BIT_SENSOR_LCD_TASK (1 << 5)  // sensor_lcd_task
#define WDOG_BIT_ALL_TASKS      (WDOG_BIT_APP_TASK | WDOG_BIT_SENSOR_TASK | \
                                 WDOG_BIT_USART_TASK | WDOG_BIT_TIANWEN_TASK | \
                                 WDOG_BIT_LCD_TASK | WDOG_BIT_SENSOR_LCD_TASK)

extern EventGroupHandle_t xWatchdogEventGroup; // 看门狗事件组

/* 全局变量声明 */
extern TaskHandle_t bspSensorTask_Handler;
extern TaskHandle_t appTask_Handler;
extern TaskHandle_t usartTask_Handler;
extern TaskHandle_t watchdogTask_Handler;
extern RingBuffer_t g_ring_buffer;
extern QueueHandle_t xUART_Queue;
extern SemaphoreHandle_t xDMA_Sem;
extern SemaphoreHandle_t xBufferDataSemaphore;

//在信号量的基础上改进为事件组，方便同时等待多个事件
// 定义系统中所有可能发生的事件
typedef enum {
    MSG_FACE_RECOG_START,      // 触发人脸识别
    MSG_FACE_RECOG_SUCCESS,    // 人脸识别成功
    MSG_FACE_RECOG_FAIL,       // 人脸识别失败
    MSG_FACE_ADD_START,        // 开始录入人脸
    MSG_FACE_ADD_SUCCESS,      // 录入人脸成功
    MSG_FACE_ADD_FAIL          // 录入人脸失败
} AppEventType_t;


// 统一的消息结构体
typedef struct {
    AppEventType_t event_id;
    uint8_t       data[4];       // 预留参数，比如可以用来传错误码或者特定的数值
} AppMsg_t;



/*esp32数据帧id号*/
#define SENSOR_SHT40    0x00
#define SENSOR_MLX90614 0x03
#define SENSOR_MAX30102 0x04
#define CMD_RTC_SYNC    0x10
#define CMD_WEATHER_SYNC 0x11
#define CMD_LOCATION_SYNC 0x12
#define CMD_WIFI_STATUS_UPDATE 0x13

/* 人脸识别结果数据帧结构 */
/* 数据位 */
/* payload[0]: 命令 (0：注册，1：识别) */
/* payload[1]: 识别结果 (0：失败, 1：成功)/注册结果（2：成功，3：失败） */
/* payload[2]: 用户ID */

extern  UI_DisplayData_t g_ui_data[MAX_USER]; // 全局 UI 数据结构实例
extern uint8_t g_current_active_user_id; // 当前活跃用户ID
// 声明全局队列句柄
extern QueueHandle_t xAppEventQueue;
/* 全局函数声明 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* watchdog task */
void watchdog_task(void *pvParameters);

#endif /* __APP_TASK_H */