/**
 * @file lv_port_disp.c
 * @brief LVGL 显示端口移植文件 - 将 LVGL 绘图接口适配到 LCD 硬件
 */

/* 将此文件复制为 "lv_port_disp.c" 并将此值设为 "1" 以使能内容 */
#if 1

/*********************
 *      头文件包含
 *********************/
#include "lv_port_disp.h"
#include <stdbool.h>
#include "lcd.h"      // LCD 显示屏驱动（窗口设置、CS/RS 控制）
#include "spi.h"      // SPI 通信驱动（数据传输）
#include "lvgl.h"     // LVGL 图形库主头文件

/*********************
 *      宏定义
 *********************/
#define MY_DISP_HOR_RES    320   /* 显示屏水平分辨率（宽度）：320 像素 */
#define MY_DISP_VER_RES    480   /* 显示屏垂直分辨率（高度）：480 像素 */

/**********************
 *      类型定义
 **********************/

/**********************
 *    静态函数声明
 **********************/
static void disp_init(void);                                    /* 显示屏初始化 */
static void disp_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p);  /* 刷新回调函数 */

/**********************
 *    静态变量
 **********************/
volatile bool disp_flush_enabled = true;   /* 刷新使能标志：true=允许刷新，false=禁止刷新 */

/**********************
 *    全局函数
 **********************/

/**
 * @brief LVGL 显示端口初始化
 * @note  注册显示驱动到 LVGL，设置显示分辨率和刷新回调函数
 */
void lv_port_disp_init(void)
{
    /* 1. 初始化显示屏硬件 */
    disp_init();

    /* 2. 初始化显示绘图缓冲区 */
    static lv_disp_draw_buf_t draw_buf_dsc;                    /* 绘图缓冲区描述符 */
    static lv_color_t buf_1[MY_DISP_HOR_RES * 2];              /* 缓冲区1：大小为 320 × 2 像素 */
    lv_disp_draw_buf_init(&draw_buf_dsc, buf_1, NULL, MY_DISP_HOR_RES * 2);  /* 双缓冲区模式（单缓冲区时第二个参数为 NULL） */

    /* 3. 初始化显示驱动并注册到 LVGL */
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);                               /* 初始化显示驱动结构体 */

    disp_drv.hor_res = MY_DISP_HOR_RES;                        /* 设置水平分辨率 */
    disp_drv.ver_res = MY_DISP_VER_RES;                        /* 设置垂直分辨率 */
    disp_drv.flush_cb = disp_flush;                            /* 绑定刷新回调函数 */
    disp_drv.draw_buf = &draw_buf_dsc;                         /* 绑定绘图缓冲区 */

    lv_disp_drv_register(&disp_drv);                           /* 注册驱动到 LVGL */
}

/**
 * @brief 使能屏幕刷新（允许 LVGL 刷新显示）
 * @note  调用此函数后，disp_flush() 将正常执行 LCD 更新操作
 */
void disp_enable_update(void)
{
    disp_flush_enabled = true;
}

/**
 * @brief 禁止屏幕刷新（LVGL 调用 disp_flush 时跳过实际刷新）
 * @note  用于需要暂时冻结画面显示的场景
 */
void disp_disable_update(void)
{
    disp_flush_enabled = false;
}

/**********************
 *    静态函数实现
 **********************/

/**
 * @brief 显示屏初始化
 * @note  LCD 已在 main.c 中完成初始化，此处留空即可
 */
static void disp_init(void)
{
    /* LCD 已在 main.c 中通过 LCD_Init() 完成初始化，此处无需额外操作 */
}

/**
 * @brief LVGL 刷新回调函数
 * @param disp_drv  LVGL 显示驱动结构体指针
 * @param area      需要刷新的区域坐标（左上角 x1,y1 到右下角 x2,y2）
 * @param color_p   待刷新区域的像素颜色数据指针（RGB565 格式）
 * @note  此函数将 LVGL 绘制的颜色数据通过 SPI 接口发送到 LCD 屏幕
 */
static void disp_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{
    /* 如果刷新被禁止，则直接通知 LVGL 刷新完成并返回 */
    if(!disp_flush_enabled) {
        lv_disp_flush_ready(disp_drv);
        return;
    }

    int32_t x, y;                              /* 循环变量：x 坐标，y 坐标 */
    uint16_t *color_16 = (uint16_t *)color_p;  /* 将颜色数据转换为 16 位 RGB565 格式指针 */

    /* 设置 LCD 的显示窗口为需要刷新的区域 */
    LCD_SetWindows(area->x1, area->y1, area->x2, area->y2);

    /* 拉低片选信号（CS），选中 LCD 进行数据传输 */
    LCD_CS_CLR;
    /* 拉高寄存器选择信号（RS），表示接下来发送的是数据（而非命令） */
    LCD_RS_SET;

    /* 遍历刷新区域内的所有像素 */
    for(y = area->y1; y <= area->y2; y++) {           /* 按行遍历 */
        for(x = area->x1; x <= area->x2; x++) {       /* 按列遍历 */
            /* 发送像素数据的高 8 位（RGB565 格式：R[4:0] + G[5:3]） */
            SPI_WriteByte(SPI1, (*color_16) >> 8);
            /* 发送像素数据的低 8 位（G[2:0] + B[4:0]） */
            SPI_WriteByte(SPI1, (*color_16) & 0xFF);
            color_16++;                               /* 指向下一个像素颜色数据 */
        }
    }

    /* 拉高片选信号（CS），结束数据传输 */
    LCD_CS_SET;

    /* 通知 LVGL 刷新操作已完成，可以继续绘制下一帧 */
    lv_disp_flush_ready(disp_drv);
}

#else /* 使能此文件：将上方的 #if 1 改为 #if 0 可禁用此移植层 */

/* 这个虚拟的 typedef 仅用于消除 -Wpedantic 警告 */
typedef int keep_pedantic_happy;
#endif