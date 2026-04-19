
#include "sht40_handler.h"

/*******************************define ************************* */
#define TEMP_HUMI_NOT_INITATED     false
#define TEMP_HUMI_INITATED         true

#define OS_QUEUE_CREATE       handler_instance->os_interface->os_queue_create
#define MY_MAX_DELAY_MS         0xFFFFFFFFUL

/*******************************Variable ************************* */
volatile bsp_temp_humi_handler_t *g_temp_humi_handler_instance = NULL;

/*****************************variable ************************* */
void _mount_handler(bsp_temp_humi_handler_t *instance)
{
      // 将实例指针保存到全局变量中，供事件处理函数访问
      g_temp_humi_handler_instance = instance;
}

//内部初始化函数：建立驱动层来联系并创建消息队列
static SHT40_HANDLER_Status_t bsp_temp_humi_handler_init(bsp_temp_humi_handler_t  *handler_instance)
{
      SHT40_Status_t sht_ret=SHT40_OK;
      SHT40_HANDLER_Status_t RET=SHT40_HANDLER_OK;

#if   SHT40_HANDLER_DEBUG
      printf("SHT40 HANDLER initialing!\r\n");
#endif
      //创建事件队列（可选）
      if(handler_instance->os_interface!=NULL&&
         handler_instance->os_interface->os_queue_create!=NULL)
      {
            RET=handler_instance->os_interface->os_queue_create(10,
                                    sizeof(temp_humi_event_t),
                                    &handler_instance->event_queue_handle);
            if(RET!=SHT40_HANDLER_OK||handler_instance->event_queue_handle==NULL)
            {
#if   SHT40_HANDLER_DEBUG
                  printf("failed to the queue create\r\n");
#endif  
                  return SHT40_HANDLER_ERROR;
            }
      }
      else
      {
            handler_instance->event_queue_handle=NULL;
      }

      //初始化底层驱动
      sht_ret=SHT40_inst(handler_instance->sht40_instance,
                         handler_instance->iic_driver_interfaces,
#ifdef OS_SUPPORTING
                        handler_instance->os_timebase_interface,
#endif
                        handler_instance->timebase_interface);
      if(SHT40_OK!=sht_ret)
      {
#if   SHT40_HANDLER_DEBUG
      printf("failed to sht40_inst\r\n");
#endif 
      return SHT40_HANDLER_ERROR;
    
      }
      return SHT40_HANDLER_OK;
}
/*************************public api************************ */
SHT40_HANDLER_Status_t bsp_temp_humi_handler_inst(
      bsp_temp_humi_handler_t             *const handler_instance,
      th_handler_input_interface_t *const input_interface)


{
#if SHT40_HANDLER_DEBUG
      printf("handler inst starting !\r\n");
#endif
      SHT40_HANDLER_Status_t RET=SHT40_HANDLER_OK;    
      //检查指针
      if(NULL==handler_instance||NULL==input_interface)
      {
#if SHT40_HANDLER_DEBUG
      printf("handler pointer is null !\r\n");
#endif
            return SHT40_HANDLER_ERROR;
      }
      if(NULL==input_interface->timebase_interface||NULL==handler_instance->private_data)
      {
#if SHT40_HANDLER_DEBUG
      printf("handler data is null !\r\n");
#endif
            return SHT40_HANDLER_ERROR;     
      }

      //依赖注入 
      handler_instance->iic_driver_interfaces=input_interface->iic_driver_interfaces;
      handler_instance->timebase_interface=input_interface->timebase_interface;
      handler_instance->os_interface=input_interface->os_interface;
      handler_instance->os_timebase_interface=input_interface->os_timebase_interface;

      handler_instance->last_temp_tick=0;
      handler_instance->last_humi_tick=0;

      //调用内部初始化函数（initial）
      RET=bsp_temp_humi_handler_init(handler_instance);
      if(RET!=SHT40_HANDLER_OK)
      {
#if SHT40_HANDLER_DEBUG
      printf("failed to handler init  !\r\n");
#endif
            return SHT40_HANDLER_ERROR;              
      }
      //更新状态并挂到全局变量中
      handler_instance->private_data->is_initiated=TEMP_HUMI_INITATED;
      _mount_handler(handler_instance);
#if SHT40_HANDLER_DEBUG
      printf("success to handler inst!\r\n");
#endif
            return SHT40_HANDLER_OK;  

      }

SHT40_HANDLER_Status_t temp_humi_process_event(bsp_temp_humi_handler_t *const handler_instance,
                                         temp_humi_event_t *const event,
                                        float * const temp, 
                                        float * const humi)
{
      uint32_t current_tick=0;
      SHT40_HANDLER_Status_t RET=SHT40_HANDLER_OK;  
      bool need_hardware_read=false;

      if(NULL==handler_instance||NULL==event)
      {
#if SHT40_HANDLER_DEBUG
      printf("process_event pointer is null!\r\n");
#endif
            return SHT40_HANDLER_ERROR;             
      }
      //检查是否初始化
      if(handler_instance->private_data->is_initiated!=TEMP_HUMI_INITATED)
      {
#if SHT40_HANDLER_DEBUG
      printf("handler is not inited!\r\n");
#endif
            return SHT40_HANDLER_ERRORSOURCE;            
      }

      //记录当前时间戳
      current_tick=handler_instance->timebase_interface->pf_get_tick_count();
      //缓存区生命周期判定逻辑
      switch(event->event_type)
      {
          case TEMP_HUMI_EVENT_TEMP:
          if((current_tick-handler_instance->last_temp_tick)>event->lifetime)
          {
            need_hardware_read=true;
          }
          break;
          case TEMP_HUMI_EVENT_HUMI:
           if((current_tick-handler_instance->last_humi_tick)>event->lifetime)
          {
            need_hardware_read=true;
          }
          break;
          case TEMP_HUMI_EVENT_BOTH:
        if((current_tick-handler_instance->last_temp_tick)>event->lifetime||
              (current_tick-handler_instance->last_humi_tick)>event->lifetime)
          {
            need_hardware_read=true;
          }
          break;

          default:  
          return SHT40_HANDLER_ERROR;

      }
      
      //开始采集数据
      if(need_hardware_read==true)
      {
            //调用底层驱动采集数据
            float *read_temp=(event->event_type==TEMP_HUMI_EVENT_TEMP||
                              event->event_type==TEMP_HUMI_EVENT_BOTH)?temp:NULL;

             float *read_humi=(event->event_type==TEMP_HUMI_EVENT_HUMI||
                              event->event_type==TEMP_HUMI_EVENT_BOTH)?humi:NULL;                  
            handler_instance->sht40_instance->pf_read(handler_instance->sht40_instance,read_temp,read_humi);
      if(read_temp)
      {
            //更新时间戳
            handler_instance->last_temp_tick=current_tick;
      }
       if(read_humi)
      {
            //更新时间戳
            handler_instance->last_humi_tick=current_tick;
      }
            need_hardware_read=false;
      }
      else
      {
#if SHT40_HANDLER_DEBUG
      printf("data with lifetime,skipping hardware i2c read!\r\n");
#endif
      }
      //执行回调函数
      if(event->pf_callback!=NULL)
      {
            event->pf_callback(temp,humi);
      }else
      {
#if SHT40_HANDLER_DEBUG
      printf("CALLBACK is NULL\r\n");
#endif           
      }
      return SHT40_HANDLER_OK;
}
            
//线程创建
void temp_humi_handler_thread(void *argument)
{
      float current_temp=0.0f;
      float current_humi=0.0f;
      SHT40_HANDLER_Status_t RET=SHT40_HANDLER_OK;
      temp_humi_event_t  current_event;
      //静态分配底层资源
      static bsp_sht40_driver_t sht40_driver_instance;
      static th_handler_private_data_t private_data={0};
      static bsp_temp_humi_handler_t handler_instance;
      th_handler_input_interface_t *input_instance=
                  (th_handler_input_interface_t *)argument;
      if(input_instance==NULL)
      {
#if SHT40_HANDLER_DEBUG
      printf("[thread] input pointer is null\r\n");
#endif 
            return ;            
      }
      //挂载并实例化
      handler_instance.sht40_instance=&sht40_driver_instance;
      handler_instance.private_data=&private_data;
      //构造
      RET=bsp_temp_humi_handler_inst(&handler_instance,input_instance);
      if(RET!=SHT40_HANDLER_OK)
      {
#if SHT40_HANDLER_DEBUG
      printf("[thread]fail to  handler inst\r\n");
#endif  
            return ;           
      }
#if SHT40_HANDLER_DEBUG
      printf("[thread]success to  thread  start!\r\n");
#endif      
//核心事件
for(;;)
{
      RET=handler_instance.os_interface->os_queue_get(
                  handler_instance.event_queue_handle,
                  &current_event,
                  MY_MAX_DELAY_MS);
      if(SHT40_HANDLER_OK==RET)
      {
            //收到数据
            RET=temp_humi_process_event(&handler_instance,&current_event,&current_temp,&current_humi);
            if(SHT40_HANDLER_OK!=RET)
            {
#if SHT40_HANDLER_DEBUG
                  printf("[thread]fail to processing event!\r\n");
#endif  

            }
      }
      else{
            vTaskDelay(10);
      }

      vTaskDelay(10);
}
}                                     