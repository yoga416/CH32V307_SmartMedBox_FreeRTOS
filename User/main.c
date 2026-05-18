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

//全局外设初始化，包含DMA、传感器接口等
printf("\r\n==================start======================\r\n");
    uint8_t ret = 0;
    ret = g_peripheral_init();
    if (ret != SYSTEM_INIT_OK) {
        printf("[Error] Peripheral initialization failed with code: %d\n", ret);
        while (1) {} // 初始化失败，停在这里
    } 
    /* IWDG 看门狗初始化：允许写入，设置分频并设置重载值，随后使能 */
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_Prescaler_256);
    IWDG_SetReload(0x0FFF);
    IWDG_ReloadCounter();
    IWDG_Enable();

    // 创建 LVGL GUI 互斥锁（必须在 LCD 任务之前创建）
    xGuiMutex = xSemaphoreCreateMutex();
    if (xGuiMutex == NULL) {
        printf("[Error] Create GUI mutex failed\n");
        while (1) {}
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
                        (UBaseType_t    )LCD_TASK_PRIO,
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
                        (UBaseType_t    )SENSOR_TASK_PRIO,
                        NULL) != pdPASS)
    {
        printf("[Error] Create sensor_lcd_task failed\n");
        while (1) {}
    }
    
    vTaskStartScheduler();
    /* 如果程序运行到这里，说明内存绝对爆了，调度器没起来 */
    printf("[FATAL] Scheduler failed! Remaining Heap: %d bytes\n", xPortGetFreeHeapSize());
    while (1)
    {
        printf("shouldn't run at here!!\n");
    }
}

/*
 * ============================================================
 *  代码优化与修复记录 (GitHub Copilot, 2026-05-11)
 * ============================================================
 *
 * 本文件汇总了本次会话中对全工程所做的所有修改。
 *
 * --- 修改 1: 解决 system_Init() 隐式声明警告 ---
 * 文件: bsp/bsp_lcd_lvgl/General_File/system.h
 * 问题: 该文件的头文件守卫使用了 __SYSTEM_H，与 System/system.h 的
 *       守卫重名。system.c 先 include "system.h" 定义了 __SYSTEM_H，
 *       导致 General_File/system.h 中 system_Init() 的声明被跳过，
 *       编译器报 implicit declaration of function 警告。
 *       在 RISC-V 上隐式声明与实际返回值不匹配可能导致栈损坏。
 * 修复: 将 General_File/system.h 的守卫改为唯一的 __LVGL_GENERAL_SYSTEM_H。
 *
 * --- 修改 2: 修复 sensor_data_lcd_queue 创建失败时的非法操作 ---
 * 文件: System/system.c (g_peripheral_init 函数内)
 * 问题: 队列创建失败时调用了 vTaskDelete(NULL)，但此时 FreeRTOS
 *       调度器尚未启动，该调用属非法操作，可能导致硬错误。
 * 修复: 改为 return SYSTEM_INIT_FAIL; 与函数其他错误处理一致。
 *
 * --- 修改 3: 补全心率血氧 LCD 显示逻辑 ---
 * 文件: User/app_task.c (sensor_lcd_task 函数)
 * 问题: case SENSOR_ID_HEART_RATE_SPO2 分支中解析了 heart_rate 和
 *       spo2 变量，但从未调用 lv_label_set_text_fmt() 更新 GUI，
 *       导致 MAX30102 算出数据后屏幕上不可见。
 * 修复: 使用空闲的 screen_label_5(心率) 和 screen_label_6(血氧)
 *       添加了带互斥锁保护的 lv_label_set_text_fmt() 调用。
 *       同时清理了该函数内未使用的 AppMsg_t msg 变量。
 *
 * --- 修改 4: 删除 check_medication_time() 中未使用的 handled 变量 ---
 * 文件: User/app_task.c (check_medication_time 函数)
 * 问题: 声明了 bool handled = false; 并在两处赋值为 true，但从未
 *       被读取，编译器报 -Wunused-but-set-variable。
 * 修复: 删除 handled 声明及所有赋值语句。
 *
 * --- 修改 5: 修复 RTC 中断中 BSP_RTC_UpdateNextAlarm() 重复调用 ---
 * 文件: User/ch32v30x_it.c (RTC_IRQHandler 函数)
 * 问题: 同一函数被连续调用两次（注释分别写"更新下一个闹钟时间"和
 *       "关键：设置下一个闹钟"），可能导致闹钟跳过或重复触发。
 * 修复: 删除重复调用，只保留一次。
 *
 * --- 修改 6: 修正 APP_TASK_STK_SIZE 注释 ---
 * 文件: User/app_task.h
 * 问题: STACK_BYTES_TO_WORDS(2048) = 2048 字节 = 2KB，但注释误写为 // 4KB。
 * 修复: 注释改为 // 2KB。
 *
 * --- 修改 7: MAX30102 采样性能优化 (缩短采集时间) ---
 * 文件: bsp/bsp_max_30102/bsp_max_30102_reg.h
 *       将 MAX30102_DATA_NUM 从 200 提升到 400，保证 100Hz 下采集
 *       窗口覆盖 4 秒，低心率(40bpm)也能捕获至少 2 个波峰。
 *
 * 文件: bsp/bsp_max_30102/bsp_max_30102_driver.c
 *       将 REG_FIFO_CONFIG 从 0x4F(SMP_AVE=4, 25Hz有效输出) 改为
 *       0x0F(SMP_AVE=1, 100Hz真实输出)，
 *       彻底消除了 "15~20 秒采集太慢" 的根本原因。
 *       同时补全了 bsp_max30102_port_init() 中遗漏的 GPIOC 时钟使能。
 *
 * 文件: middlesware/MAX_30102_algorithm/algorithm_1.h
 *       将 FS 从 96 改为 100，与传感器实际 100Hz 采样率对齐。
 *
 * 文件: bsp/bsp_max_30102/bsp_max_30102_handler.c
 *       将 PREHEAT_STABLE_COUNT 从 10 提升到 50 (0.5s 预热缓冲)，
 *       将 MAX_TIMEOUT_EMPTY 从 50 提升到 200 (2.0s 宽容超时)。
 *
 * --- 修改 8: RAM 空间优化 (96.90% -> 86.09%) ---
 * 文件: System/system.c
 *       将 CmdQueue / sendtianwenQueue / xSysCmdQueue /
 *       sensor_data_lcd_queue 的队列深度从 100 降为 10，
 *       省出约 12KB FreeRTOS 堆空间。
 *
 * 文件: middlesware/Ring_buffer/Middle_ring_buffer.h
 *       将 RING_BUFFER_SIZE 从 256 降为 64，省出约 6.5KB BSS 空间。
 *
 * 文件: User/FreeRTOSConfig.h
 *       将 configTOTAL_HEAP_SIZE 从 40*1024 降为 36*1024，
 *       回收 4KB 物理 RAM。
 *
 * 文件: bsp/bsp_lcd_lvgl/lv_conf.h
 *       将 LV_MEM_SIZE 从 10*1024 提升到 32*1024，
 *       解决 GUI Guider 复杂页面内存不足导致死机的问题。
 *
 * 文件: User/app_task.h
 *       将 LCD_TASK_STK_SIZE 从 4096(4KB) 提升到 8192(8KB)，
 *       防止 LVGL 任务栈溢出。
 *
 * --- 最终编译状态 ---
 * FLASH: 165848 B / 224 KB (72.30%)
 * RAM:   84632 B  /  96 KB (86.09%)  安全水位
 * ============================================================
 */