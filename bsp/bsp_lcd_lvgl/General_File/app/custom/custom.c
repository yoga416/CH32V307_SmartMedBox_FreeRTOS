/*
* Copyright 2023 NXP
* NXP Proprietary. This software is owned or controlled by NXP and may only be used strictly in
* accordance with the applicable license terms. By expressly accepting such terms or by downloading, installing,
* activating and/or otherwise using the software, you are agreeing that you have read, and that you agree to
* comply with and are bound by, such license terms.  If you do not agree to be bound by the applicable license
* terms, then you may not retain, install, activate or otherwise use the software.
*/


/*********************
 *      INCLUDES
 *********************/
#include <stdio.h>
#include "lvgl.h"
#include "custom.h"

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/

/**********************
 *  STATIC VARIABLES
 **********************/

/**
 * Create a demo application
 */

void custom_init(lv_ui *ui)
{
    /* Add your codes here */
}


/*
 * 通用 LV_EVENT_DELETE 回调函数。
 * 当 LVGL 销毁一个对象时触发，将 pptr 指向的指针置为 NULL。
 * 用法：lv_obj_add_event_cb(label, gui_label_delete_cb, LV_EVENT_DELETE, &guider_ui.screen_label_X);
 */
static void gui_label_delete_cb(lv_event_t *e)
{
    lv_obj_t **pptr = (lv_obj_t **)lv_event_get_user_data(e);
    if (pptr != NULL) {
        *pptr = NULL;
    }
}

void gui_protect_pointer_on_delete(lv_obj_t *obj, lv_obj_t **pptr)
{
    if (obj != NULL && pptr != NULL) {
        lv_obj_add_event_cb(obj, gui_label_delete_cb, LV_EVENT_DELETE, pptr);
    }
}

