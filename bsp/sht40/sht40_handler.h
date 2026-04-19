/******************************************************************************
 *
 * Copyright (C) 2026 SmartMedBox Team.
 * All Rights Reserved.
 *
 * @file sht40_handler.h
 *
 * @par dependencies
 * - stdint.h
 * - system.h
 * - FreeRTOS.h (如果涉及系统任务或队列调度)
 *
 * @author yoga
 *
 * @brief Provide the APIs for system event management and interrupt handling.
 *
 * This file defines the core structures, macros, and function prototypes 
 * for handling hardware interrupts, state machine transitions, and system 
 * event callbacks within the SmartMedBox application.
 *
 * @version V1.0 2026-04-06
 *
 * @note 1 tab == 4 spaces! Max 80 columns per line.
 *
 *****************************************************************************/

#ifndef __SHT40_HANDLER_H__
#define __SHT40_HANDLER_H__

/********************************INCLUDE FILES********************************/
#include <stdint.h>
#include <stdbool.h>
#include "system.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "sht40.h"

/********************************* DEFINE ************************************/
#define SHT40_HANDLER_DEBUG 1

/*********************************TYPEDEFS************************************/

/* 状态返回值枚举 */
typedef enum {
    SHT40_HANDLER_OK          = 0, /**< Operation successful */
    SHT40_HANDLER_ERROR       = 1, /**< Generic error */
    SHT40_HANDLER_ERRORSOURCE = 2  /**< Communication or sensor error */
} SHT40_HANDLER_Status_t;

/* 事件类型定义 */
typedef enum {
    TEMP_HUMI_EVENT_TEMP = 0,      /**< 仅温度事件 */
    TEMP_HUMI_EVENT_HUMI = 1,      /**< 仅湿度事件 */
    TEMP_HUMI_EVENT_BOTH = 2       /**< 温湿度同时事件 */
} temp_humi_event_type_t;

/* 温湿度事件结构体 */
typedef struct {
    float    *temperature;   /**< 温度数据指针 (注: 原文拼写, 建议为 temperature)*/
    float    *humidity;   /**< 湿度数据指针 */
    uint32_t lifetime;    /**< 事件生命周期 */
    uint32_t timestamp;   /**< 事件发生的时间戳 */
    
    temp_humi_event_type_t event_type; /**< 事件类型 */
    
    /**< 事件回调函数指针，参数为温度和湿度数据指针 */
    void (*pf_callback)(float *, float *); 
} temp_humi_event_t;

/*********************************INTERFACES**********************************/

/* OS 队列与延时操作接口 (对接系统层) */
typedef struct {
    /**< 操作系统毫秒级延时函数指针 */
    void (*os_delay_ms)(uint32_t const ms);
    
    /**< OS 队列创建接口 */
    SHT40_HANDLER_Status_t (*os_queue_create)(
        uint32_t const item_num,
        uint32_t const item_size,
        /// 注意：队列句柄是通过指针返回的，因此参数类型是 void**，调用时需要传入 &handler_instance->event_queue_handle
        void **const queue_handle);
        
    /**< OS 队列读取接口 */
    SHT40_HANDLER_Status_t (*os_queue_get)(
        void *const queue_handle,
        void *const item,
        uint32_t const timeout_ms);
        
    /**< OS 队列写入接口 */
    SHT40_HANDLER_Status_t (*os_queue_put)(
        void *const queue_handle,
        void const *const item,
        uint32_t const timeout_ms);
} th_handler_os_interface_t;

/* 处理器输入接口 (来自底层驱动层) */
typedef struct {
    iic_driver_interface_t           *iic_driver_interfaces; /**< I2C接口 */
    timebase_interface_t             *timebase_interface;    /**< 时基接口 */
    th_handler_os_interface_t *os_interface;          /**< OS层接口 */
    os_timebase_interface_t          *os_timebase_interface; /**< OS时基接口 */
} th_handler_input_interface_t;

/***********************************CLASS*************************************/

/* 私有数据结构体定义 */
typedef struct temp_humi_handler_private_data {
    bool is_initiated;
} th_handler_private_data_t;

/* Handler 核心控制块 (OOP 实例) */
typedef struct {
    timebase_interface_t             *timebase_interface;    /**< 裸机时基 */
    iic_driver_interface_t           *iic_driver_interfaces; /**< I2C驱动 */
    os_timebase_interface_t          *os_timebase_interface; /**< OS时基 */
    th_handler_os_interface_t *os_interface;          /**< OS操作接口 */
    
    bsp_sht40_driver_t               *sht40_instance;        /**< SHT40实例 */
    void                             *event_queue_handle;    /**< 事件队列句柄*/
    th_handler_private_data_t *private_data;          /**< 私有数据指针*/
    
    uint32_t last_temp_tick; /**< 上次温度事件的时间戳 */
    uint32_t last_humi_tick; /**< 上次湿度事件的时间戳 */
} bsp_temp_humi_handler_t;

/************************************API**************************************/

/**
 * @brief 温湿度处理线程入口函数
 * @param[in] argument 传递给线程的参数 (通常是 handler 实例指针)
 */
void temp_humi_handler_thread(void *argument);

/**
 * @brief 构造函数：初始化温湿度 Handler 实例
 * @note  生命周期由内部线程控制，外部无需额外管理
 * @param[in,out] handler_instance Handler 实例指针
 * @param[in]     input_interface  依赖的输入接口集合指针
 * @return SHT40_HANDLER_Status_t  状态码
 */
SHT40_HANDLER_Status_t bsp_temp_humi_handler_inst(
      bsp_temp_humi_handler_t             *const handler_instance,
      th_handler_input_interface_t *const input_interface);

/**
 * @brief 读取传感器数据并将事件推送到队列中
 * @param[in] event 指向要推送的温湿度事件结构体的指针
 * @return SHT40_HANDLER_Status_t 状态码
 */
SHT40_HANDLER_Status_t temp_humi_process_event(bsp_temp_humi_handler_t *const handler_instance,
                                         temp_humi_event_t *const event,
                                        float * const temp, 
                                        float * const humi);

extern volatile bsp_temp_humi_handler_t *g_temp_humi_handler_instance;

#endif /* end of __SHT40_HANDLER_H__ */




/****************************************************实例代码************************************** */

//在main函数中调用

#if 0
    MLX90614_Handler_start();
#endif




//在任务中调用

#if 0        //sht40_test

/////////////////////////////////////////////////////////////////////////////////////////////
#include "sht40_handler.h"
#include "sht40_port.h"

static void  SHT40_TestTask(void)
{
    //定义了事件
    temp_humi_event_t event;
    SHT40_HANDLER_Status_t RET=SHT40_HANDLER_OK;
    printf("[SHT40_TestTask] task start!\r\n");
    for(;;)
    {
        bsp_temp_humi_handler_t *handler = (bsp_temp_humi_handler_t *)g_temp_humi_handler_instance;
        event.event_type=TEMP_HUMI_EVENT_BOTH;
        event.lifetime=1000;
        event.pf_callback=on_SHT40_DATA_ready;
        if(handler!=NULL&&
           handler->os_interface!=NULL&&
           handler->event_queue_handle!=NULL)
        {
            RET=handler->os_interface->os_queue_put(
                handler->event_queue_handle,
                &event,
                10);
            if(RET!=SHT40_HANDLER_OK)
            {
                printf("[SHT40_TestTask] put queue is failed\r\n");
            }
            else
            {
                printf("[SHT40_TestTask] queue put ok\r\n");
            }
        }
        else
        {
             printf("[SHT40_TestTask] wait handler mount: inst=0x%08lX os_if=0x%08lX queue=0x%08lX\r\n",
                     (unsigned long)(uint32_t)handler,
                     (unsigned long)(uint32_t)(handler ? handler->os_interface : NULL),
                     (unsigned long)(uint32_t)(handler ? handler->event_queue_handle : NULL));
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

}

#endif