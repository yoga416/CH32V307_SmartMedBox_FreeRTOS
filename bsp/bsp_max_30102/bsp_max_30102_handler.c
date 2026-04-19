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
            switch (event.event_type)
            {
                case max_event_calc_hr_spo2: 
                {
                    // === 【第一步：彻底清空环境】 ===
                    // 1. 强制清空事件队列中积压的旧指令（防止手松开时产生的多次测量请求）
                    max_event_t dummy_event;
                    while(handler->os_handler_instance->pf_os_queue_get(
                            handler->event_queue_handler, &dummy_event, 0) == MAX_HANDLER_OK);
                    
                    // 2. 强制清空由于手松开瞬间产生的残余信号量
                    while(xSemaphoreTake(xSem_MAX30102_Exti, 0) == pdTRUE);

                    uint16_t target_samples = event.sample_count;
                    int32_t  valid_points = 0;
                    bool     is_aborted = false; // 引入放弃标志位
                    void *ctx = handler->max30102_instance->hw->hi2c;

                    // 3. 硬件寄存器复位
                    handler->max30102_instance->hw->pf_write_reg(ctx, MAX30102_I2C_ADDR, REG_FIFO_WR_PTR, 0x00);
                    handler->max30102_instance->hw->pf_write_reg(ctx, MAX30102_I2C_ADDR, REG_OVF_COUNTER, 0x00);
                    handler->max30102_instance->hw->pf_write_reg(ctx, MAX30102_I2C_ADDR, REG_FIFO_RD_PTR, 0x00);
                    
                    uint8_t dummy_status;
                    handler->max30102_instance->hw->pf_read_reg(ctx, MAX30102_I2C_ADDR, REG_INTR_STATUS_1, &dummy_status);

                    // === 【第二步：核心采集循环】 ===
                    #define PREHEAT_SAMPLES 100 
                    uint16_t total_needed = target_samples + PREHEAT_SAMPLES;

                    for (int i = 0; i < total_needed; i++) 
                    {
                        // 等待中断，超时设为 500ms
                        if (xSemaphoreTake(xSem_MAX30102_Exti, pdMS_TO_TICKS(500)) == pdTRUE) 
                        {
                            handler->max30102_instance->pf_get_filtered(handler->max30102_instance, &red, &ir);
                
                            // 清除硬件中断位
                            handler->max30102_instance->hw->pf_read_reg(ctx, MAX30102_I2C_ADDR, REG_INTR_STATUS_1, &dummy_status);

                            // --- 脱落检测 ---
                            if ((i > 20 && ir < 30000)) // 提高阈值到 8000，防止临界抖动
                            {
#ifdef MAX30102_HANDLER_DEBUG
                                printf("\r\n[Handler] Warning: Finger removed! Aborting...\n");
#endif
                                // 给用户 2 秒时间放稳手指，防止立即重新进入循环导致的连环误判
                                vTaskDelay(pdMS_TO_TICKS(2000));
                                is_aborted = true; 
                                break; 
                            }

                            // --- 预热机制 ---
                            if (i < PREHEAT_SAMPLES) continue;

                            handler->red_buffer[valid_points] = red;
                            handler->ir_buffer[valid_points]  = ir;
                            valid_points++;
                        }
                        else 
                        {
                            // 中断超时，可能是传感器连接断开
                            is_aborted = true;
                            break;
                        }
                    }

                    // === 【第三步：结果判定与清理】 ===
                    if (!is_aborted && (valid_points >= target_samples)) 
                    {
                        float spo2 = 0;
                        int32_t hr = 0;
                        int8_t spo2_valid = 0, hr_valid = 0;

                        maxim_heart_rate_and_oxygen_saturation(
                                                        handler->ir_buffer,
                                                        valid_points, 
                                                        handler->red_buffer,
                                                        &spo2, &spo2_valid, &hr, &hr_valid);

                        if (event.pf_calc_callback) {
                            event.pf_calc_callback(hr, hr_valid, spo2, spo2_valid);
                        }
                    } 
                    else 
                    {
                        // 测量失败或中途放弃：
                        // 1. 调用回调并传入 0，触发 App 层滤波器的清零逻辑
                        if (event.pf_calc_callback) event.pf_calc_callback(0, 0, 0, 0);
                        
#ifdef MAX30102_HANDLER_DEBUG
                        printf("[Handler] Logic Reset. Waiting for user stabilization...\n");
#endif
                        // 2. 强制给系统一个 2 秒的“冷静期”，防止立刻进入下一次错误的采集循环
                        vTaskDelay(pdMS_TO_TICKS(2000));
                    }
                    break;
                }
                default: break;
            }
        }
    }
}