//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它商业用途
//测试硬件：单片机CH32F103C8T6,F103C8T6最小系统开发板,主频72MHZ，晶振8MHZ
//QDtech-TFT液晶驱动 for CH32 IO模拟
//Chan@ShenZhen QDtech co.,LTD
//公司网站:www.qdtft.com
//wiki技术资料网站：http://www.lcdwiki.com
//我司提供技术支持，任何技术问题欢迎随时交流学习
//固话(传真) :+86 0755-21077707 
//手机: (销售)18823372746 （技术)15989313508
//邮箱:(销售/订单) sales@qdtft.com  (售后/技术服务)service@qdtft.com
//QQ:(售前咨询)3002706772 (技术支持)3002778157
//技术交流QQ群:778679828
//创建日期:2020/05/07
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 深圳市全动电子技术有限公司 2018-2028
//All rights reserved
/****************************************************************************************************
//=========================================电源接线================================================//
//     LCD模块                CH32单片机
//      VCC          接        DC5V/3.3V      //电源
//      GND          接          GND          //电源地
//=======================================液晶屏数据线接线==========================================//
//本模块默认数据总线类型为SPI总线
//     LCD模块                CH32单片机    
//    SDI(MOSI)      接          PA7          //液晶屏SPI总线数据写信号
//    SDO(MISO)      接          PA6          //液晶屏SPI总线数据读信号，如果不需要读，可以不接线
//=======================================液晶屏控制线接线==========================================//
//     LCD模块 					      CH32单片机 
//       LED         接          PB6          //液晶屏背光控制信号（如果不需要控制，可以不接）
//       SCK         接          PA5          //液晶屏SPI总线时钟信号
//      LCD_RS       接          PB7          //液晶屏数据/命令控制信号
//      LCD_RST      接          PB8          //液晶屏复位控制信号
//      LCD_CS       接          PB9          //液晶屏片选控制信号
//=========================================触摸屏触接线=========================================//
//如果模块不带触摸功能或者带有触摸功能，但是不需要触摸功能，则不需要进行触摸屏接线
//	   LCD模块                CH32单片机 
//     CTP_INT       接          PA8          //电容触摸屏触摸中断信号
//     CTP_SDA       接          PA9          //电容触摸屏IIC总线数据信号
//     CTP_RST       接          PA10         //电容触摸屏触摸复位信号
//     CTP_SCL       接          PB5          //电容触摸屏IIC总线时钟信号
**************************************************************************************************/	
 /* @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, QD electronic SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
**************************************************************************************************/	
#include "ft6336.h"
#include "touch.h"
#include "ctpiic.h"
#include "debug.h" 
#include "string.h" 
#include "lcd.h"

extern u8 touch_flag;

/*****************************************************************************
 * @name       :u8 FT6336_WR_Reg(u8 reg, u8 *buf, u8 len)
 * @function   :Write data to FT6336
 * @parameters :reg: Register address (8-bit)
 *              buf: Data buffer
 *              len: Length of data
 * @retvalue   :0: Success, 1: Failed
******************************************************************************/ 
u8 FT6336_WR_Reg(u8 reg, u8 *buf, u8 len)
{
    u8 i;
    u8 ret = 0;
    
    CTP_IIC_Start();
    CTP_IIC_Send_Byte(FT_CMD_WR);    // 发送写命令 0x70
    ret = CTP_IIC_Wait_Ack();
    if(ret) {
        CTP_IIC_Stop();
        return 1;
    }
    
    CTP_IIC_Send_Byte(reg);          // 发送寄存器地址
    ret = CTP_IIC_Wait_Ack();
    if(ret) {
        CTP_IIC_Stop();
        return 1;
    }
    
    for(i = 0; i < len; i++) {
        CTP_IIC_Send_Byte(buf[i]);   // 发送数据
        ret = CTP_IIC_Wait_Ack();
        if(ret) {
            CTP_IIC_Stop();
            return 1;
        }
    }
    
    CTP_IIC_Stop();
    return 0;
}

/*****************************************************************************
 * @name       :void FT6336_RD_Reg(u8 reg, u8 *buf, u8 len)
 * @function   :Read data from FT6336
 * @parameters :reg: Start register address (8-bit)
 *              buf: Data buffer
 *              len: Length of data
******************************************************************************/              
void FT6336_RD_Reg(u8 reg, u8 *buf, u8 len)
{
    u8 i;
    
    CTP_IIC_Start();
    CTP_IIC_Send_Byte(FT_CMD_WR);    // 发送写命令 0x70
    CTP_IIC_Wait_Ack();
    
    CTP_IIC_Send_Byte(reg);          // 发送寄存器地址
    CTP_IIC_Wait_Ack();
    
    CTP_IIC_Start();                 // 重新启动
    CTP_IIC_Send_Byte(FT_CMD_RD);    // 发送读命令 0x71
    CTP_IIC_Wait_Ack();
    
    for(i = 0; i < len; i++) {
        buf[i] = CTP_IIC_Read_Byte(i == (len - 1) ? 0 : 1);  // 最后一个字节发送 NACK
    }
    
    CTP_IIC_Stop();
} 

/*****************************************************************************
 * @name       :u8 FT5426_Init(void)
 * @date       :2020-05-13 
 * @function   :Initialize the ft5426 touch screen
 * @parameters :none
 * @retvalue   :0-Initialization successful
								1-initialization failed
******************************************************************************/		
u8 FT6336_Init(void)
{
    u8 temp;
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 使能 GPIOA 时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    
    // 配置触摸复位引脚 PA10
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_SetBits(GPIOA, GPIO_Pin_10);
    
    // 配置触摸中断引脚 PA8（可选）
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // I2C 初始化已在 CTP_IIC_Init() 中完成
    
    // 复位触摸芯片
    GPIO_ResetBits(GPIOA, GPIO_Pin_10);  // RST = 0
    Delay_Ms(20);
    GPIO_SetBits(GPIOA, GPIO_Pin_10);    // RST = 1
    Delay_Ms(300);
    
    // 读取芯片 ID
    FT6336_RD_Reg(FT_ID_G_FOCALTECH_ID, &temp, 1);
    printf("FT6336 ID: 0x%02X\r\n", temp);
    
    if(temp != 0x11) {
        printf("ID error! Expected 0x11\r\n");
        return 1;
    }
    
    printf("FT6336 init OK!\r\n");
    return 0;
}

const u16 FT6336_TPX_TBL[2]={FT_TP1_REG,FT_TP2_REG};

/*****************************************************************************
 * @name       :u8 FT5426_Scan(void)
 * @date       :2020-05-13 
 * @function   :Scan touch screen (query mode)
 * @parameters :none
 * @retvalue   :Current touch screen status
								0-No touch
								1-With touch
******************************************************************************/	
u8 FT6336_Scan(void)
{

    u8 buf[16];
    u8 i = 0;
    u8 res = 0;
    u8 mode = 0;
    
    // 读取触摸点的状态
    FT6336_RD_Reg(FT_REG_NUM_FINGER, &mode, 1);
    
    if(mode > 0 && mode < 7)  // 有触摸点按下
    {
        // 更新触摸状态
        tp_dev.sta = TP_PRES_DOWN | TP_CATH_PRES;
        tp_dev.sta |= mode;  // 记录触摸点数量
        
        for(i = 0; i < CTP_MAX_TOUCH; i++)
        {
            FT6336_RD_Reg(FT6336_TPX_TBL[i], buf, 4);
            
            if(i < mode)  // 有效的触摸点
            {
                // 解析坐标
                tp_dev.x[i] = ((u16)(buf[0] & 0x0F) << 8) + buf[1];
                tp_dev.y[i] = ((u16)(buf[2] & 0x0F) << 8) + buf[3];
            }
            else
            {
                tp_dev.x[i] = 0;
                tp_dev.y[i] = 0;
            }
        }
        res = 1;
    }
    else  // 无触摸
    {
        if(tp_dev.sta & TP_PRES_DOWN)
        {
            // 之前按下，现在松开了
            tp_dev.sta = 0;
            for(i = 0; i < CTP_MAX_TOUCH; i++)
            {
                tp_dev.x[i] = 0;
                tp_dev.y[i] = 0;
            }
        }
    }
    
    return res;
}
