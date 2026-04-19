#ifndef _BSP_MAX30102_HANDLER_H_
#define _BSP_MAX30102_HANDLER_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "bsp_max_30102_driver.h" 

#define MAX30102_HANDLER_DEBUG  

/* 定义算法所需的采样窗口大小 (通常需要 4~5 秒数据，100Hz = 400~500点) */
#define MAX30102_BUFFER_LENGTH  500 

typedef enum {
    MAX_HANDLER_OK          = 0,
    MAX_HANDLER_ERROR       = 1,
    MAX_HANDLER_TIMEOUT     = 2 
} MAX30102_HANDLER_Status_t;

// MAX30102 事件类型定义
typedef enum {
    max_event_read_raw       = 0,  // 读取单次原始数据
    max_event_calc_hr_spo2   = 1,  // [新增] 收集指定数量数据并解算心率血氧
    max_event_enter_sleep    = 2,  // 进入休眠
} MAX30102_HANDLER_Event_t;

// MAX30102 事件结构体
typedef struct {
    uint32_t lifetime;                 
    MAX30102_HANDLER_Event_t event_type; 
    
    uint16_t sample_count; // 需要连续收集的数据点数 
    
    // 计算完成后的回调函数指针，供事件发送者使用
    void (*pf_calc_callback)(int32_t hr, int8_t hr_valid, float spo2, int8_t spo2_valid); 
    
    // 原始数据回调函数指针，供事件发送者使用 (仅 max_event_read_raw 事件有效)
    void (*pf_raw_callback)(uint32_t red, uint32_t ir); 

} max_event_t;

/* OS 队列与延时操作接口 */
typedef struct {
    void (*pf_os_delay_ms)(uint32_t const ms);
    MAX30102_HANDLER_Status_t (*pf_os_create_queue)(uint32_t const item_num, 
                                                    uint32_t const item_size, 
                                                    void **const queue_handler);//二级指针
    MAX30102_HANDLER_Status_t (*pf_os_queue_put)(void * const queue_handler, 
                                                 void const *const item, 
                                                 uint32_t timeout_ms);
    MAX30102_HANDLER_Status_t (*pf_os_queue_get)(void * const queue_handler, 
                                                 void * const item, 
                                                 uint32_t timeout_ms);
} max_handler_os_instance_t;

typedef struct {
    bsp_max30102_driver_t     *max30102_instance;
    max_handler_os_instance_t *os_handler_instance;
} max_handler_input_instance_t;

// 处理器类定义
typedef struct {
    bsp_max30102_driver_t     *max30102_instance;
    max_handler_os_instance_t *os_handler_instance;
    // FreeRTOS 队列句柄，用于接收事件
    void *event_queue_handler; 
    
    // [核心] 将大数组放在堆/BSS段实例中，防止撑爆 FreeRTOS 任务栈
    uint32_t red_buffer[MAX30102_BUFFER_LENGTH];
    uint32_t ir_buffer[MAX30102_BUFFER_LENGTH];
    
} bsp_max_handler_t;

// 全局处理器实例指针，供中断服务程序访问
extern bsp_max_handler_t *g_max_handler_instance;

// 处理器任务函数原型
void max30102_handler_task(void *argument);

// 处理器实例化与事件发送函数原型
MAX30102_HANDLER_Status_t max30102_handler_inst(bsp_max_handler_t *handler_instance,
                                                max_handler_input_instance_t *input_instance);
// 事件发送接口，供 App 层调用
MAX30102_HANDLER_Status_t max30102_handler_send_event(bsp_max_handler_t *handler, 
                                                      max_event_t *event);

#endif /* _BSP_MAX30102_HANDLER_H_ */