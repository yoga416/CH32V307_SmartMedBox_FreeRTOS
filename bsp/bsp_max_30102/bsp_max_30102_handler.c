#include "bsp_max_30102_handler.h"
#include <stdio.h>
#include <string.h>
#include "semphr.h"
#include "algorithm_1.h" // 包含美信心率血氧算法库
//全局的信号量句柄，供中断服务程序使用
extern SemaphoreHandle_t xSem_MAX30102_Exti;
extern SemaphoreHandle_t xI2CBusMutex;
#define MAX30102_HANDLER_DEBUG
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
        printf("[Handler] Error: Failed to send event.\n");
#endif
        return MAX_HANDLER_ERROR;
    }

    return handler->os_handler_instance->pf_os_queue_put(handler->event_queue_handler, event, event->lifetime);
}

// // 处理器任务函数，负责接收事件并执行相应操作
// void max30102_handler_task(void *argument)
// {
//     bsp_max_handler_t *handler = (bsp_max_handler_t *)argument;
//     max_event_t event;
//     uint32_t red = 0, ir = 0;

//     for (;;)
//     {
//         // 1. 等待 App 指令
//         if (handler->os_handler_instance->pf_os_queue_get(
//                                         handler->event_queue_handler,
//                                         &event, portMAX_DELAY) == MAX_HANDLER_OK)
//         {
//             switch (event.event_type)
//             {
//                 case max_event_calc_hr_spo2: 
//                 {
//                     // === 【第一步：彻底清空环境】 ===
//                     // 1. 强制清空事件队列中积压的旧指令
//                     max_event_t dummy_event;
//                     while(handler->os_handler_instance->pf_os_queue_get(
//                             handler->event_queue_handler, &dummy_event, 0) == MAX_HANDLER_OK);
                    
//                     // 2. 强制清空残留的中断信号
//                     while(xSemaphoreTake(xSem_MAX30102_Exti, 0) == pdTRUE);

//                     uint16_t target_samples = event.sample_count;
//                     int32_t  valid_points = 0;
//                     bool     is_aborted = false; 
//                     void *ctx = handler->max30102_instance->hw->hi2c;

//                     // 3. 硬件寄存器复位
//                     handler->max30102_instance->hw->pf_write_reg(ctx, MAX30102_I2C_ADDR, REG_FIFO_WR_PTR, 0x00);
//                     handler->max30102_instance->hw->pf_write_reg(ctx, MAX30102_I2C_ADDR, REG_OVF_COUNTER, 0x00);
//                     handler->max30102_instance->hw->pf_write_reg(ctx, MAX30102_I2C_ADDR, REG_FIFO_RD_PTR, 0x00);
                    
//                     uint8_t dummy_status;
//                     handler->max30102_instance->hw->pf_read_reg(ctx, MAX30102_I2C_ADDR, REG_INTR_STATUS_1, &dummy_status);

//                     // === 【第二步：核心采集循环（状态机架构）】 ===
//                     #define IR_FINGER_THRESHOLD 10000 // 手指检测阈值(调低避免微小抖动误判脱落)
//                     #define PREHEAT_STABLE_COUNT 50  // 预热稳定点数 (100Hz下 50点=0.5s，让手指放稳且让滑动滤波稳定)
//                     #define MAX_TIMEOUT_EMPTY 200    // 空等超时 (200点=2.0s，增加超时宽容度)

//                     bool finger_stable = false; // 手指是否已经度过预热期
//                     int stable_count = 0;       // 预热期计数器
//                     int empty_count = 0;        // 空等超时计数器

//                     // 改用 while 循环，只有采集到足够数量的有效点才退出
//                     while (valid_points < target_samples) 
//                     {
//                         // 等待中断，超时设为 1000ms
//                         if (xSemaphoreTake(xSem_MAX30102_Exti, pdMS_TO_TICKS(500)) == pdTRUE)
//                         {
                            

//                             handler->max30102_instance->pf_get_filtered(handler->max30102_instance, &red, &ir);
                            
//                             // handler->max30102_instance->hw->pf_read_reg(ctx, MAX30102_I2C_ADDR, REG_INTR_STATUS_1, &dummy_status);
                            
// #ifdef MAX30102_HANDLER_DEBUG
//                             //APP_LOG("[Handler] Valid: %ld | Red=%lu, IR=%lu\n", valid_points, red, ir);
// #endif
                            
//                             // --- 状态机判定逻辑 ---
//                             if (ir < IR_FINGER_THRESHOLD) 
//                             {
//                                 // 状态 A：没有检测到手指
//                                 if (valid_points > 0) 
//                                 {
//                                     // 致命错误：在正式采集途中手指移开了，直接宣告测量失败
// #ifdef MAX30102_HANDLER_DEBUG
//                                     APP_LOG("[Algo Error] Finger removed during measurement!\n");
// #endif
//                                     is_aborted = true;
//                                     break;
//                                 } 
//                                 else 
//                                 {
//                                     // 还在等待手指放上去，或者手指刚放上但还在抖动（没达到预热标准）
//                                     finger_stable = false;
//                                     stable_count = 0; // 只要一低于阈值，预热计数器清零
//                                     empty_count++;

//                                     if (empty_count > MAX_TIMEOUT_EMPTY) {
//                                         // 等太久了，强制退出，防止任务死锁
//                                         APP_LOG("[Handler] Timeout waiting for finger! No valid data collected.\n");
//                                         is_aborted = true;
//                                         break;
//                                     }
//                                 }
//                             } 
//                             else 
//                             {
//                                 // 状态 B：手指在传感器上
//                                 empty_count = 0; // 重置空等计数器

//                                 if (!finger_stable) 
//                                 {
//                                     // 正在预热阶段
//                                     stable_count++;
//                                     if (stable_count >= PREHEAT_STABLE_COUNT) 
//                                     {
//                                         // 稳定期达标！正式进入采集状态
//                                         finger_stable = true;
// #ifdef MAX30102_HANDLER_DEBUG
//                                         APP_LOG("[Algo] Preheating done. Start collecting data...\n");
// #endif
//                                     }
//                                 } 
//                                 else 
//                                 {
//                                     // 正式采集阶段：存入 Buffer
//                                     handler->red_buffer[valid_points] = red;
//                                     handler->ir_buffer[valid_points]  = ir;
//                                     valid_points++;
//                                 }
//                             }
//                         }
//                         else 
//                         {
//                             // 中断超时，可能是传感器连接断开或者 I2C 死锁
//                             APP_LOG("[Handler] Timeout waiting for data! Valid points collected: %d\n", valid_points);
//                             is_aborted = true;
//                             break;
//                         }
//                     }

//                     // === 【第三步：结果判定与清理】 ===
//                     if (!is_aborted && (valid_points == target_samples)) 
//                     {
//                         float spo2 = 0;
//                         int32_t hr = 0;
//                         int8_t spo2_valid = 0, hr_valid = 0;
//                         APP_LOG("into algo: valid_points=%d\n", valid_points+1);
//                         maxim_heart_rate_and_oxygen_saturation(
//                                                 handler->ir_buffer,
//                                                 valid_points, 
//                                                 handler->red_buffer,
//                                                 &spo2, &spo2_valid, &hr, &hr_valid);

//                         if (event.pf_calc_callback) {
//                             event.pf_calc_callback(hr, hr_valid, spo2, spo2_valid);
//                         }
//                     } 
//                     else 
//                     {
//                         if (is_aborted) {
//                             APP_LOG("[Handler] failed to collect data\r\n");
//                         } 
//                         else {
//                             APP_LOG("[Handler] failed to calculate results\r\n");
//                         }

//                         // 调用回调通知失败
//                         if (event.pf_calc_callback) event.pf_calc_callback(0, 0, 0, 0);
//                         // 强制给系统一个"冷静期"，防止立刻进入下一次错误的采集循环
//                         vTaskDelay(pdMS_TO_TICKS(2000));
//                     }
//                     break;
//                 }
//             }
//         }
//     }
// }

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
                    // 1. 强制清空事件队列中积压的旧指令
                    max_event_t dummy_event;
                    while(handler->os_handler_instance->pf_os_queue_get(
                            handler->event_queue_handler, &dummy_event, 0) == MAX_HANDLER_OK);
                    
                    // 2. 强制清空残留的中断信号
                    while(xSemaphoreTake(xSem_MAX30102_Exti, 0) == pdTRUE);

                    uint16_t target_samples = event.sample_count;
                    int32_t  valid_points = 0;
                    bool     is_aborted = false; 
                    void *ctx = handler->max30102_instance->hw->hi2c;

                    // 3. 硬件寄存器复位
                    handler->max30102_instance->hw->pf_write_reg(ctx, MAX30102_I2C_ADDR, REG_FIFO_WR_PTR, 0x00);
                    handler->max30102_instance->hw->pf_write_reg(ctx, MAX30102_I2C_ADDR, REG_OVF_COUNTER, 0x00);
                    handler->max30102_instance->hw->pf_write_reg(ctx, MAX30102_I2C_ADDR, REG_FIFO_RD_PTR, 0x00);
                    
                    uint8_t dummy_status;
                    handler->max30102_instance->hw->pf_read_reg(ctx, MAX30102_I2C_ADDR, REG_INTR_STATUS_1, &dummy_status);

                    // === 【第二步：带检测的采集循环】 ===
                    #define IR_FINGER_THRESHOLD 15000 // 手指检测下限
                    #define IR_SATURATION_THRESHOLD 260000 // 饱和上限 (18位最大 262143)

#ifdef MAX30102_HANDLER_DEBUG
                    APP_LOG("[Algo] Waiting for valid finger contact...\n");
#endif

                    // 1. 预读/等待阶段：直到检测到手指且不饱和
                    uint32_t check_count = 0;
                    while (1) {
                        if (xSemaphoreTake(xSem_MAX30102_Exti, pdMS_TO_TICKS(500)) == pdTRUE) {
                            handler->max30102_instance->pf_get_filtered(handler->max30102_instance, &red, &ir);
                            handler->max30102_instance->hw->pf_read_reg(ctx, MAX30102_I2C_ADDR, REG_INTR_STATUS_1, &dummy_status);
                            
                            if (ir > IR_FINGER_THRESHOLD && ir < IR_SATURATION_THRESHOLD) {
                                break; // 检测到有效手指接触
                            }
                            
                            if (++check_count > 100) { // 5秒超时
                                APP_LOG("[Handler] No finger or saturated! Aborting.\n");
                                is_aborted = true;
                                break;
                            }
                        } else {
                            is_aborted = true;
                            break;
                        }
                    }

                    if (!is_aborted) {
                        APP_LOG("[Algo] Finger detected. Start collecting %d data points...\n", target_samples);
                    }

                    TickType_t start_tick = xTaskGetTickCount();
                    // 2. 正式采集阶段
                    while (!is_aborted && valid_points < target_samples) 
                    {
                        // 等待中断，超时设为 500ms
                        if (xSemaphoreTake(xSem_MAX30102_Exti, pdMS_TO_TICKS(500)) == pdTRUE)
                        {
                            // 读取数据
                            handler->max30102_instance->pf_get_filtered(handler->max30102_instance, &red, &ir);
                            handler->max30102_instance->hw->pf_read_reg(ctx, MAX30102_I2C_ADDR, REG_INTR_STATUS_1, &dummy_status);
                            
                            // 采集途中如果手指移开或饱和，立即终止
                            if (ir < IR_FINGER_THRESHOLD || ir > IR_SATURATION_THRESHOLD) {
                                APP_LOG("[Handler] Contact lost or saturated during collection!\n");
                                is_aborted = true;
                                break;
                            }

                            // 直接存入 Buffer
                            handler->red_buffer[valid_points] = red;
                            handler->ir_buffer[valid_points]  = ir;
                            valid_points++;
                        }
                        else 
                        {
                            // 500ms都没来中断，说明传感器死锁或断开了
                            APP_LOG("[Handler] Timeout during collection! Valid points: %d\n", valid_points);
                            is_aborted = true;
                            break;
                        }
                    }
                    TickType_t end_tick = xTaskGetTickCount();
                    uint32_t duration_ms = (end_tick - start_tick) * (1000 / configTICK_RATE_HZ);
                    float actual_fs = (duration_ms > 0) ? (valid_points * 1000.0f / duration_ms) : 100.0f;

                    // === 【第三步：结果判定与清理】 ===
                    if (!is_aborted && (valid_points == target_samples)) 
                    {
                        float spo2 = 0;
                        int32_t hr = 0;
                        int8_t spo2_valid = 0, hr_valid = 0;
                        APP_LOG("into algo: valid_points=%d, actual_fs=%.2f Hz\n", valid_points, actual_fs);

                        // 把采集到的 400 多个点全扔给官方算法
                        maxim_heart_rate_and_oxygen_saturation(
                                        handler->ir_buffer,
                                        valid_points, 
                                        handler->red_buffer,
                                        &spo2, &spo2_valid, &hr, &hr_valid);
                        
                        // 血氧校准：用户反馈偏低，增加 1-2% 的偏移量
                        if (spo2_valid && spo2 > 0) {
                            spo2 -= 1.5f; 
                            if (spo2 > 100.0f) spo2 = 100.0f;
                        }
                        
                        APP_LOG("algo done: HR=%d (valid=%d), SpO2=%.1f%% (valid=%d)\n", hr, hr_valid, spo2, spo2_valid);
                        // /*打印原始数据*/
                        // for (int i = 0; i < valid_points; i++) {
                        //     APP_LOG("Sample %d: Red=%d, IR=%d\n", i, handler->red_buffer[i], handler->ir_buffer[i]);
                        // }
                        // 将结果传给 App 层，此时如果数据是纯噪声（没放手指），算法计算出的 hr_valid 会是 0
                        if (event.pf_calc_callback) {
                            event.pf_calc_callback(hr, hr_valid, spo2, spo2_valid);
                        }
                    } 
                    else 
                    {
                        if (is_aborted) {
                            APP_LOG("[Handler] failed to collect data\r\n");
                        } 
                        else {
                            APP_LOG("[Handler] failed to calculate results\r\n");
                        }

                        // 调用回调通知失败
                        if (event.pf_calc_callback) event.pf_calc_callback(0, 0, 0, 0);
                        // 强制给系统一个"冷静期"
                        vTaskDelay(pdMS_TO_TICKS(2000));
                    }
                    break;
                }
            }
        }
    }
}