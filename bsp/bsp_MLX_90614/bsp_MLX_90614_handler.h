#ifndef _BSP_MLX_90614_HANDLER_H_
#define _BSP_MLX_90614_HANDLER_H_

///***********************************INCLUDES***********************************/
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "system.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
/************************************DEFINES************************************/
#define MLX90614_HANDLER_DEBUG  1
/************************************TYPEDEFS***********************************/
//状态返回值枚举
typedef enum {
    MLX90614_HANDLER_OK          = 0, /**< Operation successful */
    MLX90614_HANDLER_ERROR       = 1, /**< Generic error */
    MLX90614_HANDLER_ERRORSOURCE = 2  /**< Communication or sensor error */
} MLX90614_HANDLER_Status_t;

//事件类型定义
typedef enum{
     th_event_surface_temp = 0,
     th_event_body_temp = 1,
      th_event_both_temp = 2,
} MLX90614_HANDLER_Event_t;

//温度事件结构体
typedef struct {
      float    *surface_temperature;   /**< 表面温度数据指针 */
      float    *body_temperature;      /**< 体温数据指针 */
      uint32_t lifetime;               /**< 事件生命周期 */
      uint32_t timestamp;              /**< 事件发生的时间戳 */
      
      MLX90614_HANDLER_Event_t event_type; /**< 事件类型 */
      
      /**< 事件回调函数指针，参数为表面温度和体温数据指针 */
      void (*pf_callback)(float *, float *); 

}th_event_t;
/////////////////////interfaces///////////////////
/* OS 队列与延时操作接口 (对接系统层) */
typedef struct{
      //延迟函数
      void (*pf_os_delay_ms)(uint32_t  const ms );

      MLX90614_Status_t (* pf_os_create_queue)(
                  uint32_t const  item_num,
                  uint32_t const item_size,
                  //用二级指针去传句柄
                  void **const queue_handler
      );

      MLX90614_Status_t (* pf_os_queue_put)(
                  void * const queue_handler,
                  void const *const item,
                  uint32_t timeout_ms
      );

         MLX90614_Status_t (* pf_os_queue_get)(
                  void * const queue_handler,
                  void * const item,
                  uint32_t timeout_ms
      );
      
}th_handler_os_instance_t;

//处理器输入接口
typedef struct {
      bsp_mlx90614_driver_t *mlx90614_instance;
      mlx_iic_driver_instance_t *iic_driver_interface;
      mlx_timebase_interface_t *timebase_interface;
      mlx_os_timebase_interface_t *os_timebase_interface;
      th_handler_os_instance_t *os_handler_instance;
}th_handler_input_instance_t;

///////////////////class////////////////////////////
//前向声明
// 1. 仅仅是类型的前向声明 (Forward Declaration)
typedef struct th_handler_private_data th_mlx90614_private_data_t;

//bsp_th_handler类定义
typedef struct {
      mlx_iic_driver_instance_t *iic_driver_interface; /**< I2C接口 */
      mlx_timebase_interface_t *timebase_interface;    /**< 时基接口 */     
      mlx_os_timebase_interface_t *os_timebase_interface; /**< OS时基接口 */
      
      //os_queue
      th_handler_os_instance_t *os_handler_instance;
      //私有数据指针
      th_mlx90614_private_data_t *private_data;

      //事件队列句柄
      void *event_queue_handler;
      
      //mlx90614实例指针
      bsp_mlx90614_driver_t *mlx90614_instance;
      
      //时间戳
      uint32_t last_surface_tick;
      uint32_t last_body_tick;

}bsp_th_handler_t;

extern bsp_th_handler_t *g_handler_instance;

///////////////api///////////////////////
//线程入口函数
  void th_handler_task(void * argument);

  //构造函数
  MLX90614_HANDLER_Status_t  th_handler_inst(
      bsp_th_handler_t *handler_instance,
      th_handler_input_instance_t *input_instance
  );

  //发送温度事件函数
  MLX90614_HANDLER_Status_t th_handler_send_event(
      bsp_th_handler_t *handler_instance,
      th_event_t *event,
      float * const surface_temp, 
       float * const body_temp);

#endif



/////////////////////////////实例代码////////////////////////////




//在main函数中调用

#if 0
sht40_handler_start();
#endif 
//在任务中调用
#if 0        //mlx90614_test_driver

static void MLX90614_TestTask(void)
{
   //定义了事件
    th_event_t event;
    MLX90614_HANDLER_Status_t RET=MLX90614_HANDLER_OK;
    printf("[MLX90614_TestTask] task start!\r\n");
    for(;;)
    {
        bsp_th_handler_t *handler = (bsp_th_handler_t *)g_handler_instance;
        event.event_type=th_event_body_temp;
        event.lifetime=1000;
        event.pf_callback=MLX90614_Port_OnDataReady;
        if(handler!=NULL&&
           handler->os_handler_instance!=NULL&&
           handler->event_queue_handler!=NULL)
        {
            RET=handler->os_handler_instance->pf_os_queue_put(
                handler->event_queue_handler,
                &event,
                10);
            if(RET!=MLX90614_HANDLER_OK)
            {
                printf("[MLX90614_TestTask] put queue is failed\r\n");
            }
            else
            {
                printf("[MLX90614_TestTask] queue put ok\r\n");
            }
        }
        else
        {
             printf("[MLX90614_TestTask] wait handler mount: inst=0x%08lX os_if=0x%08lX queue=0x%08lX\r\n",
                     (unsigned long)(uint32_t)handler,
                     (unsigned long)(uint32_t)(handler ? handler->os_handler_instance : NULL),
                     (unsigned long)(uint32_t)(handler ? handler->event_queue_handler : NULL));
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

#endif