/* delay.h - 为 max30102.c 提供 delay_us/delay_ms 函数声明（映射到项目已有的 Delay_Us/Delay_Ms） */
#ifndef _MAX30102_DELAY_H_
#define _MAX30102_DELAY_H_

#include "debug.h"          // 提供 Delay_Us / Delay_Ms 声明
#define delay_us(n)  Delay_Us(n)
#define delay_ms(n)  Delay_Ms(n)

#endif /* _MAX30102_DELAY_H_ */
