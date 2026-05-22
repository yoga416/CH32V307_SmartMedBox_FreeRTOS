#include "system.h"
#include "debug.h"
#include "ch32v30x_gpio.h"

#ifndef GPIO_TogglePin
#define GPIO_TogglePin(GPIOx, GPIO_Pin) ((GPIOx)->OUTDR ^= (GPIO_Pin))
#endif

void system_Init()
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE,ENABLE);

    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1 |GPIO_Pin_2 |GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;

    GPIO_Init(GPIOE,&GPIO_InitStructure);
}

void system_Loop()
{
    static uint32_t cnt = 0;
    cnt++;
    if(cnt >= 500) {  // ? 500 ?????????????????????
        cnt = 0;
        GPIO_TogglePin(GPIOE, GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3);
    }
}