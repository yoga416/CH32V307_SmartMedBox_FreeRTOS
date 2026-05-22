#include "ctpiic.h"
#include "debug.h"	 
#include "FreeRTOS.h"
#include "task.h"

/*****************************************************************************
 * @name       :void CTP_Delay(void)
 * @date       :2020-05-13 
 * @function   :Delay in controlling IIC speed
 * @parameters :None
 * @retvalue   :None
******************************************************************************/
void CTP_Delay(void)
{
    // 使用纯软件循环实现约 1.5us - 2us 的延时
    // 避免在任务中使用 Delay_Us 重置 SysTick 导致调度器崩溃
    volatile uint32_t i = 120;
    while(i--);
} 

/*****************************************************************************
 * @name       :void CTP_IIC_Init(void)
 * @date       :2020-05-13 
 * @function   :Initialize IIC
 * @parameters :None
 * @retvalue   :None
******************************************************************************/
void CTP_IIC_Init(void)
{   
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 使能 GPIOB 时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    
    // 配置 SCL (PB10) 和 SDA (PB11) 为开漏输出
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // 初始化为高电平
    GPIO_SetBits(GPIOB, GPIO_Pin_10);   // SCL = 1
    GPIO_SetBits(GPIOB, GPIO_Pin_11);   // SDA = 1
}

/*****************************************************************************
 * @name       :void CTP_IIC_Start(void)
 * @date       :2020-05-13 
 * @function   :Generating IIC starting signal
 * @parameters :None
 * @retvalue   :None
******************************************************************************/
void CTP_IIC_Start(void)
{
    CTP_SDA_OUT();                      // sda线输出
    GPIO_SetBits(GPIOB, GPIO_Pin_11);   // SDA = 1
    GPIO_SetBits(GPIOB, GPIO_Pin_10);   // SCL = 1
    CTP_Delay();
    GPIO_ResetBits(GPIOB, GPIO_Pin_11); // START: when CLK is high, DATA change from high to low
    CTP_Delay();
    GPIO_ResetBits(GPIOB, GPIO_Pin_10); // 钳住I2C总线，准备发送或接收数据
}   

/*****************************************************************************
 * @name       :void CTP_IIC_Stop(void)
 * @date       :2020-05-13 
 * @function   :Generating IIC stop signal
 * @parameters :None
 * @retvalue   :None
******************************************************************************/   
void CTP_IIC_Stop(void)
{ 
    CTP_SDA_OUT();                      // sda线输出
    GPIO_ResetBits(GPIOB, GPIO_Pin_10); // SCL = 0
    GPIO_ResetBits(GPIOB, GPIO_Pin_11); // SDA = 0
    CTP_Delay();
    GPIO_SetBits(GPIOB, GPIO_Pin_10);   // SCL = 1
    CTP_Delay();
    GPIO_SetBits(GPIOB, GPIO_Pin_11);   // STOP: when CLK is high, DATA change from low to high
}

/*****************************************************************************
 * @name       :u8 CTP_IIC_Wait_Ack(void)
 * @date       :2020-05-13 
 * @function   :Wait for the response signal
 * @parameters :None
 * @retvalue   :0-receive response signal successfully
 *             1-receive response signal unsuccessfully
******************************************************************************/ 
u8 CTP_IIC_Wait_Ack(void)
{
    u8 ucErrTime = 0;
    CTP_SDA_IN();                       // SDA设置为输入
    GPIO_SetBits(GPIOB, GPIO_Pin_11);   // SDA = 1
    CTP_Delay();
    GPIO_SetBits(GPIOB, GPIO_Pin_10);   // SCL = 1
    CTP_Delay();
    while(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11))
    {
        ucErrTime++;
        if(ucErrTime > 250)
        {
            CTP_IIC_Stop();
            return 1;
        }
    }
    GPIO_ResetBits(GPIOB, GPIO_Pin_10); // 时钟输出0
    return 0;
} 

/*****************************************************************************
 * @name       :void CTP_IIC_Ack(void)
 * @date       :2020-05-13 
 * @function   :Generate ACK response signal
 * @parameters :None
 * @retvalue   :None
******************************************************************************/ 
void CTP_IIC_Ack(void)
{
    GPIO_ResetBits(GPIOB, GPIO_Pin_10); // SCL = 0
    CTP_SDA_OUT();
    GPIO_ResetBits(GPIOB, GPIO_Pin_11); // SDA = 0
    CTP_Delay();
    GPIO_SetBits(GPIOB, GPIO_Pin_10);   // SCL = 1
    CTP_Delay();
    GPIO_ResetBits(GPIOB, GPIO_Pin_10); // SCL = 0
}

/*****************************************************************************
 * @name       :void CTP_IIC_NAck(void)
 * @date       :2020-05-13 
 * @function   :Don't generate ACK response signal
 * @parameters :None
 * @retvalue   :None
******************************************************************************/        
void CTP_IIC_NAck(void)
{
    GPIO_ResetBits(GPIOB, GPIO_Pin_10); // SCL = 0
    CTP_SDA_OUT();
    GPIO_SetBits(GPIOB, GPIO_Pin_11);   // SDA = 1
    CTP_Delay();
    GPIO_SetBits(GPIOB, GPIO_Pin_10);   // SCL = 1
    CTP_Delay();
    GPIO_ResetBits(GPIOB, GPIO_Pin_10); // SCL = 0
}   

/*****************************************************************************
 * @name       :void CTP_IIC_Send_Byte(u8 txd)
 * @date       :2020-05-13 
 * @function   :send a byte data by IIC bus
 * @parameters :txd:Data to be sent
 * @retvalue   :None
******************************************************************************/                     
void CTP_IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
    /* [RTOS] 移除 vTaskSuspendAll：本函数仅 ~16us 耗时的位碰撞，
       无需挂起整个调度器。此前在 LVGL 回调中挂起调度器会直接
       导致 vTaskDelay/lv_timer_handler 无法运行而触发 HardFault。
       触摸 I2C 已移出 LVGL 回调，在普通任务中执行，任务切换
       产生的字节间延迟 FT6336 完全可容忍。 */
    CTP_SDA_OUT();      
    GPIO_ResetBits(GPIOB, GPIO_Pin_10); // 拉低时钟开始数据传输
    for(t = 0; t < 8; t++)
    {              
        if((txd & 0x80) >> 7)
            GPIO_SetBits(GPIOB, GPIO_Pin_11);
        else
            GPIO_ResetBits(GPIOB, GPIO_Pin_11);
        txd <<= 1;          
        GPIO_SetBits(GPIOB, GPIO_Pin_10);
        CTP_Delay();
        GPIO_ResetBits(GPIOB, GPIO_Pin_10);  
        CTP_Delay();
    }  
}   

/*****************************************************************************
 * @name       :u8 CTP_IIC_Read_Byte(unsigned char ack)
 * @date       :2020-05-13 
 * @function   :read a byte data by IIC bus
 * @parameters :ack:0-send nACK, 1-send ACK
 * @retvalue   :Data to be read
******************************************************************************/        
u8 CTP_IIC_Read_Byte(unsigned char ack)
{
    u8 i, receive = 0;
    /* [RTOS] 移除 vTaskSuspendAll：原因同 CTP_IIC_Send_Byte，
       ~16us 位碰撞无需挂起调度器。 */
    CTP_SDA_IN();                       // SDA设置为输入
    for(i = 0; i < 8; i++)
    {
        GPIO_ResetBits(GPIOB, GPIO_Pin_10);      
        CTP_Delay();
        GPIO_SetBits(GPIOB, GPIO_Pin_10);  
        receive <<= 1;
        if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11))
            receive++;
        CTP_Delay();
    }
    GPIO_ResetBits(GPIOB, GPIO_Pin_10); // 拉低时钟
    if(!ack)
        CTP_IIC_NAck();                 // 发送nACK
    else
        CTP_IIC_Ack();                  // 发送ACK   
    //xTaskResumeAll(); // 恢复调度
    return receive;
}