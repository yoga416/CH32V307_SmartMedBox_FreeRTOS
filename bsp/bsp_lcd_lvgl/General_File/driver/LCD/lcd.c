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
#include "lcd.h"
#include "stdlib.h"
#include "debug.h"		 
#include "spi.h"
#include "font.h" 

	   
//管理LCD重要参数
//默认为竖屏
_lcd_dev lcddev;

//画笔颜色,背景颜色
u16 POINT_COLOR = 0x0000,BACK_COLOR = 0xFFFF;  
u16 DeviceCode;	 

/*****************************************************************************
 * @name       :void LCD_WR_REG(u8 data)
 * @date       :2018-08-09 
 * @function   :Write an 8-bit command to the LCD screen
 * @parameters :data:Command value to be written
 * @retvalue   :None
******************************************************************************/
void LCD_WR_REG(u8 data)
{ 
   LCD_CS_CLR;     
	 LCD_RS_CLR;	  
   SPI_WriteByte(SPI1,data);
   LCD_CS_SET;	
}

/*****************************************************************************
 * @name       :void LCD_WR_DATA(u8 data)
 * @date       :2018-08-09 
 * @function   :Write an 8-bit data to the LCD screen
 * @parameters :data:data value to be written
 * @retvalue   :None
******************************************************************************/
void LCD_WR_DATA(u8 data)
{
   LCD_CS_CLR;
	 LCD_RS_SET;
   SPI_WriteByte(SPI1,data);
   LCD_CS_SET;
}

u8 LCD_RD_DATA(void)
{
	 u8 data;
	 LCD_CS_CLR;
	 LCD_RS_SET;
	 SPI_SetSpeed(SPI1,0);
	 data = SPI_WriteByte(SPI1,0xFF);
	 SPI_SetSpeed(SPI1,1);
	 LCD_CS_SET;
	 return data;
}

/*****************************************************************************
 * @name       :void LCD_WriteReg(u8 LCD_Reg, u16 LCD_RegValue)
 * @date       :2018-08-09 
 * @function   :Write data into registers
 * @parameters :LCD_Reg:Register address
                LCD_RegValue:Data to be written
 * @retvalue   :None
******************************************************************************/
void LCD_WriteReg(u8 LCD_Reg, u16 LCD_RegValue)
{	
	LCD_WR_REG(LCD_Reg);  
	LCD_WR_DATA(LCD_RegValue);	    		 
}	   

u8 LCD_ReadReg(u8 LCD_Reg)
{
	LCD_WR_REG(LCD_Reg);
  return LCD_RD_DATA();
}

/*****************************************************************************
 * @name       :void LCD_WriteRAM_Prepare(void)
 * @date       :2018-08-09 
 * @function   :Write GRAM
 * @parameters :None
 * @retvalue   :None
******************************************************************************/	 
void LCD_WriteRAM_Prepare(void)
{
	LCD_WR_REG(lcddev.wramcmd);
}	 


void LCD_ReadRAM_Prepare(void)
{
	LCD_WR_REG(lcddev.rramcmd);
}	 

/*****************************************************************************
 * @name       :void Lcd_WriteData_16Bit(u16 Data)
 * @date       :2018-08-09 
 * @function   :Write an 16-bit command to the LCD screen
 * @parameters :Data:Data to be written
 * @retvalue   :None
******************************************************************************/	 
void Lcd_WriteData_16Bit(u16 Data)
{	
   LCD_CS_CLR;
   LCD_RS_SET;  
	SPI_WriteByte(SPI1,Data>>8);
	SPI_WriteByte(SPI1,Data);

   LCD_CS_SET;
}

u16 Lcd_ReadData_16Bit(void)
{
	u16 r,g;
	LCD_CS_CLR;
	LCD_RS_CLR;	  
	SPI_WriteByte(SPI1,lcddev.rramcmd);
	SPI_SetSpeed(SPI1,0);
	LCD_RS_SET;
	SPI_WriteByte(SPI1,0xFF);
	r = SPI_WriteByte(SPI1,0xFF);
	g = SPI_WriteByte(SPI1,0xFF);
	SPI_SetSpeed(SPI1,1);
	LCD_CS_SET;
	r<<=8;
	r|=g;
	return r;
}

/*****************************************************************************
 * @name       :void LCD_DrawPoint(u16 x,u16 y)
 * @date       :2018-08-09 
 * @function   :Write a pixel data at a specified location
 * @parameters :x:the x coordinate of the pixel
                y:the y coordinate of the pixel
 * @retvalue   :None
******************************************************************************/	
void LCD_DrawPoint(u16 x,u16 y)
{
	LCD_SetCursor(x,y);//设置光标位置 
	Lcd_WriteData_16Bit(POINT_COLOR); 
}

u16 LCD_ReadPoint(u16 x,u16 y)
{
	u16 color;
	LCD_SetCursor(x,y);//设置光标位置 
	color = Lcd_ReadData_16Bit();
	return color;
}

/*****************************************************************************
 * @name       :void LCD_Clear(u16 Color)
 * @date       :2018-08-09 
 * @function   :Full screen filled LCD screen
 * @parameters :color:Filled color
 * @retvalue   :None
******************************************************************************/	
void LCD_Clear(u16 Color)
{
  unsigned int i,m;  
	LCD_SetWindows(0,0,lcddev.width-1,lcddev.height-1);   
	LCD_CS_CLR;
	LCD_RS_SET;
	for(i=0;i<lcddev.height;i++)
	{
    for(m=0;m<lcddev.width;m++)
    {	
			SPI_WriteByte(SPI1,Color>>8);
			SPI_WriteByte(SPI1,Color);
		}
	}
	 LCD_CS_SET;
} 

/*****************************************************************************
 * @name       :void LCD_GPIOInit(void)
 * @date       :2018-08-09 
 * @function   :Initialization LCD screen GPIO
 * @parameters :None
 * @retvalue   :None
******************************************************************************/	
void LCD_GPIOInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 使能 GPIOA 时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    
    // 配置 PA1(背光), PA2(RST), PA3(DC), PA4(CS)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // 背光点亮
    GPIO_SetBits(GPIOA, GPIO_Pin_1);  // 或者用 LCD_LED_SET
}

/*****************************************************************************
 * @name       :void LCD_RESET(void)
 * @date       :2018-08-09 
 * @function   :Reset LCD screen
 * @parameters :None
 * @retvalue   :None
******************************************************************************/	
void LCD_RESET(void)
{
	LCD_RST_CLR;
	Delay_Ms(100);	
	LCD_RST_SET;
	Delay_Ms(50);
}

/*****************************************************************************
 * @name       :void LCD_RESET(void)
 * @date       :2018-08-09 
 * @function   :Initialization LCD screen
 * @parameters :None
 * @retvalue   :None
******************************************************************************/	 	 
void LCD_Init(void)
{  
	SPI1_Init(); //硬件SPI1初始化
	LCD_GPIOInit();//LCD GPIO初始化										 
//如果mcu的复位引脚没有引出来，则接PB8，调用下列复位函数	
 	LCD_RESET(); //LCD 复位
//*************3.5 ST7796S IPS初始化**********//	
	LCD_WR_REG(0x11);     

	Delay_Ms(120);                //Delay 120ms

	LCD_WR_REG(0x36);     // Memory Data Access Control MY,MX~~
	LCD_WR_DATA(0x48);   

	LCD_WR_REG(0x3A);     
	LCD_WR_DATA(0x55);   

	LCD_WR_REG(0xF0);     // Command Set Control
	LCD_WR_DATA(0xC3);   

	LCD_WR_REG(0xF0);     
	LCD_WR_DATA(0x96);   

	LCD_WR_REG(0xB4);     
	LCD_WR_DATA(0x02);   

	LCD_WR_REG(0xB7);     
	LCD_WR_DATA(0xC6);   

	LCD_WR_REG(0xC0);     
	LCD_WR_DATA(0xC0);   
	LCD_WR_DATA(0x00);   

	LCD_WR_REG(0xC1);     
	LCD_WR_DATA(0x13);   

	LCD_WR_REG(0xC2);     
	LCD_WR_DATA(0xA7);   

	LCD_WR_REG(0xC5);     
	LCD_WR_DATA(0x21);   

	LCD_WR_REG(0xE8);     
	LCD_WR_DATA(0x40);   
	LCD_WR_DATA(0x8A);   
	LCD_WR_DATA(0x1B);   
	LCD_WR_DATA(0x1B);   
	LCD_WR_DATA(0x23);   
	LCD_WR_DATA(0x0A);   
	LCD_WR_DATA(0xAC);   
	LCD_WR_DATA(0x33);   

	LCD_WR_REG(0xE0);     
	LCD_WR_DATA(0xD2);   
	LCD_WR_DATA(0x05);   
	LCD_WR_DATA(0x08);   
	LCD_WR_DATA(0x06);   
	LCD_WR_DATA(0x05);   
	LCD_WR_DATA(0x02);   
	LCD_WR_DATA(0x2A);   
	LCD_WR_DATA(0x44);   
	LCD_WR_DATA(0x46);   
	LCD_WR_DATA(0x39);   
	LCD_WR_DATA(0x15);   
	LCD_WR_DATA(0x15);   
	LCD_WR_DATA(0x2D);   
	LCD_WR_DATA(0x32);   

	LCD_WR_REG(0xE1);     
	LCD_WR_DATA(0x96);   
	LCD_WR_DATA(0x08);   
	LCD_WR_DATA(0x0C);   
	LCD_WR_DATA(0x09);   
	LCD_WR_DATA(0x09);   
	LCD_WR_DATA(0x25);   
	LCD_WR_DATA(0x2E);   
	LCD_WR_DATA(0x43);   
	LCD_WR_DATA(0x42);   
	LCD_WR_DATA(0x35);   
	LCD_WR_DATA(0x11);   
	LCD_WR_DATA(0x11);   
	LCD_WR_DATA(0x28);   
	LCD_WR_DATA(0x2E);   

	LCD_WR_REG(0xF0);     
	LCD_WR_DATA(0x3C);   
	LCD_WR_REG(0xF0);     
	LCD_WR_DATA(0x69);  

	Delay_Ms(120);                
	LCD_WR_REG(0x21);     
	LCD_WR_REG(0x29);     

  LCD_direction(USE_HORIZONTAL);//设置LCD显示方向 
	LCD_Clear(WHITE);//清全屏白色
}
 
/*****************************************************************************
 * @name       :void LCD_SetWindows(u16 xStar, u16 yStar,u16 xEnd,u16 yEnd)
 * @date       :2018-08-09 
 * @function   :Setting LCD display window
 * @parameters :xStar:the bebinning x coordinate of the LCD display window
								yStar:the bebinning y coordinate of the LCD display window
								xEnd:the endning x coordinate of the LCD display window
								yEnd:the endning y coordinate of the LCD display window
 * @retvalue   :None
******************************************************************************/ 
void LCD_SetWindows(u16 xStar, u16 yStar,u16 xEnd,u16 yEnd)
{	
	LCD_WR_REG(lcddev.setxcmd);	
	LCD_WR_DATA(xStar>>8);
	LCD_WR_DATA(0x00FF&xStar);		
	LCD_WR_DATA(xEnd>>8);
	LCD_WR_DATA(0x00FF&xEnd);

	LCD_WR_REG(lcddev.setycmd);	
	LCD_WR_DATA(yStar>>8);
	LCD_WR_DATA(0x00FF&yStar);		
	LCD_WR_DATA(yEnd>>8);
	LCD_WR_DATA(0x00FF&yEnd);

	LCD_WriteRAM_Prepare();	//开始写入GRAM			
}   

/*****************************************************************************
 * @name       :void LCD_SetCursor(u16 Xpos, u16 Ypos)
 * @date       :2018-08-09 
 * @function   :Set coordinate value
 * @parameters :Xpos:the  x coordinate of the pixel
								Ypos:the  y coordinate of the pixel
 * @retvalue   :None
******************************************************************************/ 
void LCD_SetCursor(u16 Xpos, u16 Ypos)
{	  	    			
	LCD_SetWindows(Xpos,Ypos,Xpos,Ypos);	
} 

/*****************************************************************************
 * @name       :void LCD_direction(u8 direction)
 * @date       :2018-08-09 
 * @function   :Setting the display direction of LCD screen
 * @parameters :direction:0-0 degree
                          1-90 degree
													2-180 degree
													3-270 degree
 * @retvalue   :None
******************************************************************************/ 
void LCD_direction(u8 direction)
{ 
			lcddev.setxcmd=0x2A;
			lcddev.setycmd=0x2B;
			lcddev.wramcmd=0x2C;
			lcddev.rramcmd=0x2E;
			lcddev.dir = direction%4;
	switch(lcddev.dir){		  
		case 0:						 	 		
			lcddev.width=LCD_W;
			lcddev.height=LCD_H;		
			LCD_WriteReg(0x36,(1<<3)|(1<<6));
		break;
		case 1:
			lcddev.width=LCD_H;
			lcddev.height=LCD_W;
			LCD_WriteReg(0x36,(1<<3)|(1<<5));
		break;
		case 2:						 	 		
			lcddev.width=LCD_W;
			lcddev.height=LCD_H;	
			LCD_WriteReg(0x36,(1<<3)|(1<<7));
		break;
		case 3:
			lcddev.width=LCD_H;
			lcddev.height=LCD_W;
			LCD_WriteReg(0x36,(1<<3)|(1<<7)|(1<<6)|(1<<5));
		break;	
		default:break;
	}	
}	 

u16 LCD_Read_ID(void)
{
	u8 i,val[3] = {0};
	LCD_WR_REG(0xF0);     // Command Set Control
	LCD_WR_DATA(0xC3);   

	LCD_WR_REG(0xF0);     
	LCD_WR_DATA(0x96);  
	LCD_CS_CLR;
	for(i=1;i<4;i++)
	{
		LCD_RS_CLR;	  
		SPI_WriteByte(SPI1,0xFB);
		LCD_RS_SET;
		SPI_WriteByte(SPI1,0x10+i);
		LCD_RS_CLR;	  
		SPI_WriteByte(SPI1,0xD3);
		SPI_SetSpeed(SPI1,0);
		LCD_RS_SET;
		val[i-1] = SPI_WriteByte(SPI1,0xFF);
		SPI_SetSpeed(SPI1,1);
		LCD_RS_CLR;	  
		SPI_WriteByte(SPI1,0xFB);
		LCD_RS_SET;
		SPI_WriteByte(SPI1,0x00);	
	}
	LCD_CS_SET;
	LCD_WR_REG(0xF0);     // Command Set Control
	LCD_WR_DATA(0x3C);   
	LCD_WR_REG(0xF0);     
	LCD_WR_DATA(0x69);  
	lcddev.id=val[1];
	lcddev.id<<=8;
	lcddev.id|=val[2];
	return lcddev.id;
}

// 显示一个字符（修正镜像问题）
void LCD_ShowChar(u16 x, u16 y, u8 ch, u16 color, u16 bgcolor, u8 size)
{
    u8 temp;
    u8 i, j;
    const unsigned char *p;
    
    if(size == 12) {
        p = asc2_1206[ch - 32];
        for(i = 0; i < 12; i++) {
            temp = p[i];
            for(j = 0; j < 6; j++) {
                if(temp & 0x01) {        // 改为检查最低位
                    LCD_DrawPoint(x + j, y + i);
                }
                temp >>= 1;              // 改为右移
            }
        }
    } else if(size == 16) {
        p = asc2_1608[ch - 32];
        for(i = 0; i < 16; i++) {
            temp = p[i];
            for(j = 0; j < 8; j++) {
                if(temp & 0x01) {        // 改为检查最低位
                    LCD_DrawPoint(x + j, y + i);
                }
                temp >>= 1;              // 改为右移
            }
        }
    }
}

// 显示字符串 - 注意 x += size/2 是向右移动
void LCD_ShowString(u16 x, u16 y, const u8 *str, u16 color, u16 bgcolor, u8 size)
{
    while(*str) {
        LCD_ShowChar(x, y, *str, color, bgcolor, size);
        x += size / 2;  
        str++;
    }
}
/*****************************************************************************
 * @name       :void LCD_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2)
 * @date       :2018-08-09 
 * @function   :Draw a line between two points
 * @parameters :x1,y1: start point
 *             x2,y2: end point
 * @retvalue   :None
******************************************************************************/
void LCD_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2)
{
    u16 t;
    int xerr = 0, yerr = 0, delta_x, delta_y, distance;
    int incx, incy, uRow, uCol;
    
    delta_x = x2 - x1;
    delta_y = y2 - y1;
    uRow = x1;
    uCol = y1;
    
    if(delta_x > 0)
        incx = 1;
    else if(delta_x == 0)
        incx = 0;
    else
    {
        incx = -1;
        delta_x = -delta_x;
    }
    
    if(delta_y > 0)
        incy = 1;
    else if(delta_y == 0)
        incy = 0;
    else
    {
        incy = -1;
        delta_y = -delta_y;
    }
    
    if(delta_x > delta_y)
        distance = delta_x;
    else
        distance = delta_y;
    
    for(t = 0; t <= distance + 1; t++)
    {
        LCD_DrawPoint(uRow, uCol);
        xerr += delta_x;
        yerr += delta_y;
        if(xerr > distance)
        {
            xerr -= distance;
            uRow += incx;
        }
        if(yerr > distance)
        {
            yerr -= distance;
            uCol += incy;
        }
    }
}