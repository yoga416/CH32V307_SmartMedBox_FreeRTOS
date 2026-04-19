#include "bsp_battery_port.h"
#include "ch32v30x.h"
#include "bsp_battery_driver.h"
#include "bsp_battery_handler.h"
#include <stdlib.h>

#define BAT_ADC_CH             ADC_Channel_4       
#define BAT_ADC_GPIO_PORT      GPIOA
#define BAT_ADC_GPIO_PIN       GPIO_Pin_4          

// 高精度采样配置
#define BAT_SAMPLE_COUNT       50
#define BAT_DISCARD_COUNT      10

bsp_battery_driver_t g_battery_driver;

/* qsort 排序回调 */
static int cmp_uint16(const void *a, const void *b) {
    return (*(uint16_t*)a - *(uint16_t*)b);
}

/* ---------------- 50次中位值平均滤波读取 ---------------- */
static uint16_t hw_read_stable_adc(void) {
    uint16_t adc_buffer[BAT_SAMPLE_COUNT];
    uint32_t valid_sum = 0;
    uint8_t  valid_count = BAT_SAMPLE_COUNT - (BAT_DISCARD_COUNT * 2);

    for(uint8_t i = 0; i < BAT_SAMPLE_COUNT; i++) {
        ADC_RegularChannelConfig(ADC1, BAT_ADC_CH, 1, ADC_SampleTime_239Cycles5);
        ADC_SoftwareStartConvCmd(ADC1, ENABLE);
        while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)); 
        adc_buffer[i] = ADC_GetConversionValue(ADC1);
        
        // 使用 OS 延时，每次让出 2ms CPU，总采样时长跨越 100ms
        vTaskDelay(pdMS_TO_TICKS(2)); 
    }

    // 排序并切除极值
    qsort(adc_buffer, BAT_SAMPLE_COUNT, sizeof(uint16_t), cmp_uint16);

    for(uint8_t i = BAT_DISCARD_COUNT; i < (BAT_SAMPLE_COUNT - BAT_DISCARD_COUNT); i++) {
        valid_sum += adc_buffer[i];
    }

    return (uint16_t)(valid_sum / valid_count);
}

/* ---------------- OS 延时适配 ---------------- */
static void os_delay(uint32_t const ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

/* ---------------- 底层外设初始化 ---------------- */
void bsp_battery_port_init(void) {
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    ADC_InitTypeDef  ADC_InitStructure  = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1, ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div8); 

    GPIO_InitStructure.GPIO_Pin   = BAT_ADC_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN; 
    GPIO_Init(BAT_ADC_GPIO_PORT, &GPIO_InitStructure);

    ADC_DeInit(ADC1);
    ADC_InitStructure.ADC_Mode               = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode       = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv   = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign          = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel       = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_Cmd(ADC1, ENABLE);

    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));

    // 挂载硬件接口
    static bat_hw_interface_t hw_if = {
        .pf_read_stable_adc = hw_read_stable_adc
    };
    bsp_battery_inst(&g_battery_driver, &hw_if);
}

/* ---------------- 启动业务处理 Task ---------------- */
void bsp_battery_task_init(void) {
    static bat_handler_os_instance_t os_if = {
        .pf_os_delay_ms = os_delay
    };

    bsp_battery_handler_inst(&g_bat_handler_instance, &g_battery_driver, &os_if);

    // 创建后台电池任务
    xTaskCreate(battery_handler_task, 
                "Battery_Task", 
                256, 
                &g_bat_handler_instance, 
                1,   // 最低优先级运行即可，不抢占主业务
                NULL);
}


/***************************start battery_handler_task config function***************************/
 //bsp_battery_port_init();
 //bsp_battery_task_init();