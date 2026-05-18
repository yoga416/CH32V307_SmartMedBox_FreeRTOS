#ifndef __CTPIIC_H
#define __CTPIIC_H
#include "debug.h"	    

// IIC IO方向设置（使用 PB11 作为 SDA）
#define CTP_SDA_IN()  {GPIO_InitTypeDef GPIO_InitStruct; GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11; GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU; GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; GPIO_Init(GPIOB, &GPIO_InitStruct);}
#define CTP_SDA_OUT() {GPIO_InitTypeDef GPIO_InitStruct; GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11; GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_OD; GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; GPIO_Init(GPIOB, &GPIO_InitStruct);}

// IO操作函数（SCL=PB10, SDA=PB11）
#define CTP_IIC_SCL_SET()   GPIO_SetBits(GPIOB, GPIO_Pin_10)
#define CTP_IIC_SCL_CLR()   GPIO_ResetBits(GPIOB, GPIO_Pin_10)
#define CTP_IIC_SDA_SET()   GPIO_SetBits(GPIOB, GPIO_Pin_11)
#define CTP_IIC_SDA_CLR()   GPIO_ResetBits(GPIOB, GPIO_Pin_11)
#define CTP_READ_SDA        GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11)

// IIC所有操作函数
void CTP_IIC_Init(void);
void CTP_IIC_Start(void);
void CTP_IIC_Stop(void);
void CTP_IIC_Send_Byte(u8 txd);
u8 CTP_IIC_Read_Byte(unsigned char ack);
u8 CTP_IIC_Wait_Ack(void);
void CTP_IIC_Ack(void);
void CTP_IIC_NAck(void);

#endif