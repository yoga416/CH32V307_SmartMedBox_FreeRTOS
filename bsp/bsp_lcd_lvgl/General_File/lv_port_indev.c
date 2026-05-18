/**
 * @file lv_port_indev.c
 */

#if 1

/*********************
 *      INCLUDES
 *********************/
#include "lv_port_indev.h"
#include "lvgl.h"
#include "touch.h"
#include "debug.h"

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void touchpad_read(lv_indev_drv_t * indev_drv, lv_indev_data_t * data);

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void lv_port_indev_init(void)
{
    static lv_indev_drv_t indev_drv;
    
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = touchpad_read;
    lv_indev_drv_register(&indev_drv);
    
    printf("lv_port_indev_init done!\r\n");
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

static void touchpad_read(lv_indev_drv_t * indev_drv, lv_indev_data_t * data)
{
    static uint32_t call_count = 0;
    call_count++;
    
    // 每 200 次打印一次，确认函数被调用
    if(call_count % 200 == 1) {
        printf("touchpad_read called: %d times\r\n", call_count);
    }
    static lv_coord_t last_x = 0, last_y = 0;
    
    // 扫描触摸屏
    FT6336_Scan();
    
    extern _m_tp_dev tp_dev;
    
    // 调试打印
    static uint32_t print_cnt = 0;
    print_cnt++;
    if(print_cnt >= 30) {
        print_cnt = 0;
        printf("Touch state: sta=0x%02X, x=%d, y=%d\r\n", 
               tp_dev.sta, tp_dev.x[0], tp_dev.y[0]);
    }
    
    if(tp_dev.sta & TP_PRES_DOWN) {
        // 原始坐标
        lv_coord_t raw_x = tp_dev.x[0];
        lv_coord_t raw_y = tp_dev.y[0];
        
        // 180度旋转映射（屏幕分辨率 320x480）
        lv_coord_t mapped_x = 320 - raw_x;
        lv_coord_t mapped_y = 480 - raw_y;
        
        // 限制坐标范围
        if(mapped_x < 0) mapped_x = 0;
        if(mapped_x >= 320) mapped_x = 319;
        if(mapped_y < 0) mapped_y = 0;
        if(mapped_y >= 480) mapped_y = 479;
        
        // 只在坐标变化时打印
        if(last_x != mapped_x || last_y != mapped_y) {
            printf("Touch: raw(%d,%d) -> mapped(%d,%d)\r\n", raw_x, raw_y, mapped_x, mapped_y);
        }
        
        last_x = mapped_x;
        last_y = mapped_y;
        data->state = LV_INDEV_STATE_PRESSED;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
    
    data->point.x = last_x;
    data->point.y = last_y;
}

#endif