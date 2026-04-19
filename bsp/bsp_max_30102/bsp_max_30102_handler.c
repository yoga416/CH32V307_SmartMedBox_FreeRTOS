#include "bsp_max_30102_handler.h"
#include <stdio.h>
#include <string.h>
#include "semphr.h"
#include "algorithm_1.h" // 包含美信心率血氧算法库

//全局的信号量句柄，供中断服务程序使用
extern SemaphoreHandle_t xSem_MAX30102_Exti;

//实例化一个全局处理器实例指针，供中断服务程序访问
bsp_max_handler_t *g_max_handler_instance = NULL;

//构造函数，初始化处理器实例
MAX30102_HANDLER_Status_t max30102_handler_inst(bsp_max_handler_t *handler_instance, 
                                                max_handler_input_instance_t *input_instance)
{
    if (handler_instance == NULL || input_instance == NULL) 
    {
#ifdef MAX30102_HANDLER_DEBUG
        printf("[Handler] Error: Invalid input to max30102_handler_inst.\n");
#endif
       return MAX_HANDLER_ERROR;
    }
    // 初始化处理器实例内存，并关联驱动实例和 OS 接口实例
    memset(handler_instance, 0, sizeof(bsp_max_handler_t));
    // 关联驱动实例和 OS 接口实例(input_instance 由 App 层创建并传入，包含了驱动实例和 OS 接口实例的指针)
    handler_instance->max30102_instance   = input_instance->max30102_instance;
    handler_instance->os_handler_instance = input_instance->os_handler_instance;
    // 创建事件队列，假设队列长度为 10，元素大小为 max_event_t
    MAX30102_HANDLER_Status_t ret = handler_instance->os_handler_instance->pf_os_create_queue(
                                     10, 
                                     sizeof(max_event_t), 
                                     &handler_instance->event_queue_handler);
                                    
    if (ret == MAX_HANDLER_OK) 
    {
#ifdef MAX30102_HANDLER_DEBUG
        printf("[Handler] Event queue created successfully.\n");
 #endif
 // 将实例指针赋值给全局变量，供中断服务程序访问
        g_max_handler_instance = handler_instance;
    }
    return ret;
}

MAX30102_HANDLER_Status_t max30102_handler_send_event(bsp_max_handler_t *handler, 
                                                      max_event_t *event)
{

    if (!handler || !handler->os_handler_instance || !handler->event_queue_handler)
    {
#ifdef MAX30102_HANDLER_DEBUG
        printf("send event error\n");
#endif
        return MAX_HANDLER_ERROR;
    }

    return handler->os_handler_instance->pf_os_queue_put(handler->event_queue_handler, event, event->lifetime);
}

// 处理器任务函数，负责接收事件并执行相应操作
void max30102_handler_task(void *argument)
{
    bsp_max_handler_t *handler = (bsp_max_handler_t *)argument;
    max_event_t event;
    uint32_t red = 0, ir = 0;

    for (;;)
    {
        // 1. 等待 App 指令
        if (handler->os_handler_instance->pf_os_queue_get(
                                        handler->event_queue_handler,
                                        &event, portMAX_DELAY) == MAX_HANDLER_OK)
        {
            // 2. 根据事件类型执行相应操作
            switch (event.event_type)
            {
                // 多次读取原始数据事件，适用于需要连续采集数据的算法解算场景
                case max_event_calc_hr_spo2: 
                {
                    uint16_t target_samples = event.sample_count;
                    int32_t  valid_points = 0;
                    void *ctx = handler->max30102_instance->hw->hi2c;

                    // --- A. 采集前强制复位硬件，确保引脚处于高电平 ---
                    handler->max30102_instance->hw->pf_write_reg(ctx, MAX30102_I2C_ADDR, REG_FIFO_WR_PTR, 0x00);
                    handler->max30102_instance->hw->pf_write_reg(ctx, MAX30102_I2C_ADDR, REG_OVF_COUNTER, 0x00);
                    handler->max30102_instance->hw->pf_write_reg(ctx, MAX30102_I2C_ADDR, REG_FIFO_RD_PTR, 0x00);
                    
                    uint8_t dummy;
                    handler->max30102_instance->hw->pf_read_reg(ctx, MAX30102_I2C_ADDR, REG_INTR_STATUS_1, &dummy);
                    xQueueReset(xSem_MAX30102_Exti);

                    // --- B. 核心采集循环 (必须快！) ---
                    for (int i = 0; i < target_samples; i++) 
                    {
                        // 等待中断，超时时间设为 500ms 足够
                        if (xSemaphoreTake(xSem_MAX30102_Exti, pdMS_TO_TICKS(500)) == pdTRUE) 
                        {

                        handler->max30102_instance->pf_get_filtered(handler->max30102_instance, &red, &ir);
            
                         uint8_t status_reg_1;
                         // 读取中断状态寄存器，清除中断 (如果之前的 pf_get_filtered 没有自动清除中断的话，这里确保清除)
                        handler->max30102_instance->hw->pf_read_reg(ctx, MAX30102_I2C_ADDR, REG_INTR_STATUS_1, &status_reg_1);
                        //打印滤波后的数据
#ifdef MAX30102_HANDLER_DEBUG
                        printf("[Handler] Sample %d: Red=%lu, IR=%lu\r\n", i+1, red, ir);
#endif
                              // 4. 脱落检测
                              if (ir < 5000) // 这个阈值需要根据实际情况调整，过高可能误判脱落，过低可能漏掉有效数据
                              {
#ifdef MAX30102_HANDLER_DEBUG
                         printf("\r\n[Handler] Warning: Finger removed!\n");
#endif
                              break; 
                            }

                              handler->red_buffer[i] = red;
                              handler->ir_buffer[i]  = ir;
                              valid_points++;
                              }
                    }
                    // --- C. 数据收集完成，执行算法解算 ---
                    if (valid_points >= target_samples) 
                    {
                        // 调用美信算法库进行心率血氧解算
                        float spo2 = 0;
                        int32_t hr = 0;
                        int8_t spo2_valid = 0, hr_valid = 0;
                        // 注意：算法库函数参数类型需要与定义保持一致
                        maxim_heart_rate_and_oxygen_saturation(
                                                        handler->ir_buffer,
                                                        valid_points, 
                                                        handler->red_buffer,
                                                        &spo2, 
                                                        &spo2_valid, 
                                                        &hr,    
                                                        &hr_valid
                                                        );

                        if (event.pf_calc_callback) {
                            event.pf_calc_callback(hr, hr_valid, spo2, spo2_valid);
                        }
                        else{
#ifdef MAX30102_HANDLER_DEBUG
                            printf("[Handler] callback is NULL\n");
#endif // DEBUG_ENABLE                      
                            }
                    } 
                    else {
#ifdef MAX30102_HANDLER_DEBUG
                    printf("[Handler] Collection failed (incomplete data).\n");
#endif // DEBUG_ENABLE
                        if (event.pf_calc_callback) event.pf_calc_callback(0, 0, 0, 0);
                    }
                    break;
                }
                default: break;
            }
        }
    }
}