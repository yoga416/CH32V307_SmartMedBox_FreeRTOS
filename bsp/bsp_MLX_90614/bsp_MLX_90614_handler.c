#include <stdio.h>

#include "bsp_MLX_90614_handler.h"

//define
#define HANDLER_INITIATED true
#define HANDLER_NOT_INITIATED false
#define MY_MAX_DELAY_MS 0xFFFFFFFFUL

//全局变量
bsp_th_handler_t *g_handler_instance = NULL;

//指针挂载函数
void _mount_handler_mlx(bsp_th_handler_t *instance)
{
      g_handler_instance = instance;
}

// 这里的结构体标签必须与头文件的前向声明一致
struct th_handler_private_data {
     bool is_initited; 
};
//内部初始化函数：建立驱动层来联系并创建消息队列
static MLX90614_HANDLER_Status_t th_handler_init(
                  bsp_th_handler_t *handler_instance)
{
      MLX90614_Status_t RET_dr = MLX90614_OK;

      if (handler_instance == NULL)
      {
#if MLX90614_HANDLER_DEBUG
            printf("handler_instance pointer is null\r\n");
#endif
            return MLX90614_HANDLER_ERROR;
      }

      if (handler_instance->os_handler_instance == NULL ||
          handler_instance->os_handler_instance->pf_os_create_queue == NULL)
      {
#if MLX90614_HANDLER_DEBUG
            printf("os_create_queue pointer is null\r\n");
#endif
            return MLX90614_HANDLER_ERROR;
      }

      if (handler_instance->mlx90614_instance == NULL ||
          handler_instance->iic_driver_interface == NULL ||
          handler_instance->timebase_interface == NULL)
      {
#if MLX90614_HANDLER_DEBUG
            printf(" [mlx_90614] dependency instance is NULL!\r\n");
#endif
            return MLX90614_HANDLER_ERROR;
      }

      //创建队列
      RET_dr = handler_instance->os_handler_instance->pf_os_create_queue(
                  10,
                  sizeof(th_event_t),
                  &handler_instance->event_queue_handler);
      if (RET_dr != MLX90614_OK || handler_instance->event_queue_handler == NULL)
      {
#if MLX90614_HANDLER_DEBUG
            printf("failed to queue_create!\r\n");
#endif
            return MLX90614_HANDLER_ERROR;
      }

      //初始化mlx90614底层代码
      RET_dr = bsp_mlx_90614_inst(
            handler_instance->mlx90614_instance,
            handler_instance->iic_driver_interface,
#ifdef OS_SUPPORTING
            handler_instance->os_timebase_interface,
#endif
            handler_instance->timebase_interface);
      if (RET_dr != MLX90614_OK)
      {
#if MLX90614_HANDLER_DEBUG
            printf(" [mlx_90614] BSP_MLX90614 INIT FAILED!\r\n");
#endif
            return MLX90614_HANDLER_ERROR;
      }

      return MLX90614_HANDLER_OK;
}

/*********************************PUBLIC API*********************************** */
 //构造函数
  MLX90614_HANDLER_Status_t  th_handler_inst(
      bsp_th_handler_t *handler_instance,
      th_handler_input_instance_t *input_instance
  )
  {

      MLX90614_HANDLER_Status_t RET=MLX90614_HANDLER_OK;
#if MLX90614_HANDLER_DEBUG
      printf("th_handler_inst starting!\r\n");
#endif

      if(handler_instance==NULL||input_instance==NULL)
      {
#if MLX90614_HANDLER_DEBUG
      printf("[handler_inst] fail to insatnce pointer is NULL!\r\n");
#endif  
      return MLX90614_HANDLER_ERROR;   
      }
      if(input_instance->timebase_interface==NULL)
      {
#if MLX90614_HANDLER_DEBUG
      printf("[handler_inst] fail to insatnce pointer is NULL!\r\n");
#endif  
      return MLX90614_HANDLER_ERROR;     
}
      //挂载结构体
      handler_instance->mlx90614_instance=input_instance->mlx90614_instance;
      handler_instance->iic_driver_interface=input_instance->iic_driver_interface;
      handler_instance->os_timebase_interface=input_instance->os_timebase_interface;
      handler_instance->timebase_interface=input_instance->timebase_interface;
      handler_instance->os_handler_instance=input_instance->os_handler_instance;

      handler_instance->last_surface_tick=0;
      handler_instance->last_body_tick=0;

      //初始化handler函数
      RET=th_handler_init(handler_instance);
      if(RET!=MLX90614_HANDLER_OK)

      {
#if MLX90614_HANDLER_DEBUG
      printf("[handler_inst] fail to handler init!\r\n");
#endif       
      return MLX90614_HANDLER_ERROR;
      }

      //更新全局变量
      if(handler_instance->private_data==NULL)
      {
#if MLX90614_HANDLER_DEBUG
      printf("[handler_inst] private_data pointer is NULL!\r\n");
#endif
      return MLX90614_HANDLER_ERROR;
      }
      handler_instance->private_data->is_initited=HANDLER_INITIATED;
      _mount_handler_mlx(handler_instance);
#if MLX90614_HANDLER_DEBUG
      printf("[handler_inst] success to inst!\r\n");
#endif  
      return MLX90614_HANDLER_OK;
  }

MLX90614_HANDLER_Status_t th_handler_send_event(
      bsp_th_handler_t *handler_instance,
      th_event_t *event,
      float * const surface_temp, 
       float * const body_temp)
  {
      uint32_t current_tick=0;
      bool need_hardware_read=false;
//检查指针
if(handler_instance==NULL||event==NULL)
{
#if MLX90614_HANDLER_DEBUG
      printf("[handler_event] pointer is null!\r\n");
#endif  
      return MLX90614_HANDLER_ERROR;     
}
if(handler_instance->private_data==NULL)
{
#if MLX90614_HANDLER_DEBUG
      printf("[handler_event] private_data is null!\r\n");
#endif
      return MLX90614_HANDLER_ERROR;
}
//检查是否初始化
if(handler_instance->private_data->is_initited!=HANDLER_INITIATED)
{
#if MLX90614_HANDLER_DEBUG
      printf("[handler_event] handler is not inited!\r\n");
#endif 
      return MLX90614_HANDLER_ERROR;   
}
//记录当前时间戳
      current_tick=handler_instance->timebase_interface->pf_get_ms(NULL);
//缓冲区判断
switch (event->event_type)
{
      case th_event_surface_temp:
      //检查新鲜度
      if((current_tick-handler_instance->last_surface_tick)>event->lifetime)
      {
      //需要重新检测
      need_hardware_read=true;      
      }
      break;

      case th_event_body_temp:
      if((current_tick-handler_instance->last_body_tick)>event->lifetime)
      {
      //需要重新检测
      need_hardware_read=true;      
      }
      break;

      default:
      return MLX90614_HANDLER_ERROR;
}

      //检查是否需要重新测量
      if(need_hardware_read==true)
      {
            if(event->event_type==th_event_surface_temp)
            {
                  handler_instance->mlx90614_instance->pf_read_surface_temp(handler_instance->mlx90614_instance,surface_temp);
            }

            if(event->event_type==th_event_body_temp)
            {
                  handler_instance->mlx90614_instance->pf_read_body_temp(handler_instance->mlx90614_instance,body_temp);
            }

            //检查数据
            if(surface_temp)
            {
                  //更新时间戳
                  handler_instance->last_surface_tick=current_tick;
            }
            //检查数据
            if(body_temp)
            {
                  //更新时间戳
                  handler_instance->last_body_tick=current_tick;
            }
      }
      else
      {
#if MLX90614_HANDLER_DEBUG
            printf("the lifetime is good, not renew data\r\n");
#endif
      }
      if(event->pf_callback!=NULL)
      {
      event->pf_callback(surface_temp,body_temp);
      }
      else{
#if MLX90614_HANDLER_DEBUG
            printf("pf_callback is null\r\n");
#endif  
return  MLX90614_HANDLER_ERROR;        
      }
      return MLX90614_HANDLER_OK;
  }

//线程入口函数
void th_handler_task(void * argument)
{
      float current_surface_temp=0;
      float current_body_temp=0;
      MLX90614_HANDLER_Status_t RET_handler=MLX90614_OK;  

      th_event_t    current_event;
      //静态分配资源
      bsp_mlx90614_driver_t mlx_driver;
      th_mlx90614_private_data_t private_data={0};
      th_handler_input_instance_t *input_instance=
            (th_handler_input_instance_t *) argument;
      bsp_th_handler_t hander_instance={0};
      

      //检查
      if(input_instance==NULL)
      {
#if MLX90614_HANDLER_DEBUG
            printf("[mlx_handler_thread]  instance is null\r\n");
#endif  
            return;
      }
      hander_instance.mlx90614_instance = &mlx_driver;
      hander_instance.private_data = &private_data;

      //构造函数
      RET_handler=th_handler_inst(&hander_instance,input_instance);
      if(RET_handler!=MLX90614_HANDLER_OK)
      {
#if MLX90614_HANDLER_DEBUG
            printf("[mlx_handler_thread]  failed to inst\r\n");
#endif  
            return;
      }
      else if(RET_handler==MLX90614_HANDLER_OK)
      {
#if MLX90614_HANDLER_DEBUG
            printf("[mlx_handler_thread] success to  thread  start!\r\n");
#endif        
      }

      for(;;)
      {
      RET_handler= hander_instance.os_handler_instance->pf_os_queue_get(
                                    hander_instance.event_queue_handler,
                                          &current_event,MY_MAX_DELAY_MS);
      if(RET_handler==MLX90614_HANDLER_OK)
      {
            //收到数据
            RET_handler=th_handler_send_event(&hander_instance,&current_event,
                                                &current_surface_temp,
                                                &current_body_temp);        
      }
      else
      {
            vTaskDelay(10);
      }
            vTaskDelay(10);
      }
}