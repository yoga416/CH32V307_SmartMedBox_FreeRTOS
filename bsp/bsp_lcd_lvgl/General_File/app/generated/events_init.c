/*
* Copyright 2026 NXP
* NXP Proprietary. This software is owned or controlled by NXP and may only be used strictly in
* accordance with the applicable license terms. By expressly accepting such terms or by downloading, installing,
* activating and/or otherwise using the software, you are agreeing that you have read, and that you agree to
* comply with and are bound by, such license terms.  If you do not agree to be bound by the applicable license
* terms, then you may not retain, install, activate or otherwise use the software.
*/

#include "events_init.h"
#include <stdio.h>
#include <stdlib.h>
#include "lvgl.h"
#include "information.h"

#if LV_USE_GUIDER_SIMULATOR && LV_USE_FREEMASTER
#include "freemaster_client.h"
#endif

/*药品在最开始的时候没有数据*/
extern AlarmSche my_meds[3];
/* * 统一的全局指针清理函数
 * 每次离开当前屏幕前调用此函数，彻底切断所有旧控件与后台任务的联系，防止野指针导致的 HardFault 
 */
static void clear_all_old_pointers(void)
{
    // === 主屏幕 (screen) ===
    guider_ui.screen_cont = NULL;
    guider_ui.screen_cont_1 = NULL;
    guider_ui.screen_label_1 = NULL;
    guider_ui.screen_label_2 = NULL;
    guider_ui.screen_label_4 = NULL;
    guider_ui.screen_label_5 = NULL;
    guider_ui.screen_label_6 = NULL;
    guider_ui.screen_label_7 = NULL;
    guider_ui.screen_label_8 = NULL;
    guider_ui.screen_btn_1 = NULL;
    guider_ui.screen_btn_1_label = NULL;
    guider_ui.screen_label_3 = NULL;

    // === 屏幕1 (screen_1) ===
    guider_ui.screen_1_cont_1 = NULL;
    guider_ui.screen_1_btn_1 = NULL;
    guider_ui.screen_1_btn_1_label = NULL;
    guider_ui.screen_1_btn_2 = NULL;
    guider_ui.screen_1_btn_2_label = NULL;
    guider_ui.screen_1_btn_3 = NULL;
    guider_ui.screen_1_btn_3_label = NULL;
    guider_ui.screen_1_btn_4 = NULL;
    guider_ui.screen_1_btn_4_label = NULL;

    // === 屏幕2 (screen_2) ===
    guider_ui.screen_2_cont_1 = NULL;
    guider_ui.screen_2_label_1 = NULL;
    guider_ui.screen_2_label_2 = NULL;
    guider_ui.screen_2_label_3 = NULL;
    guider_ui.screen_2_label_4 = NULL;
    guider_ui.screen_2_label_5 = NULL;
    guider_ui.screen_2_label_6 = NULL;
    guider_ui.screen_2_label_10 = NULL;
    guider_ui.screen_2_label_11 = NULL;
    guider_ui.screen_2_label_12 = NULL;
    guider_ui.screen_2_label_13 = NULL;
    guider_ui.screen_2_label_14 = NULL;
    guider_ui.screen_2_label_15 = NULL;
    guider_ui.screen_2_btn_1 = NULL;
    guider_ui.screen_2_btn_1_label = NULL;

    // === 屏幕3 (screen_3) ===
    guider_ui.screen_3_cont_1 = NULL;
    guider_ui.screen_3_label_1 = NULL;
    guider_ui.screen_3_btn_1 = NULL;
    guider_ui.screen_3_btn_1_label = NULL;

    // === 屏幕4 (screen_4) ===
    guider_ui.screen_4_cont_1 = NULL;
    guider_ui.screen_4_label_1 = NULL;
    guider_ui.screen_4_label_2 = NULL;
    guider_ui.screen_4_label_3 = NULL;
    guider_ui.screen_4_ddlist_2 = NULL;
    guider_ui.screen_4_ddlist_3 = NULL;
    guider_ui.screen_4_btn_1 = NULL;
    guider_ui.screen_4_btn_1_label = NULL;
    guider_ui.screen_4_label_21 = NULL;
    guider_ui.screen_4_label_20 = NULL;
    guider_ui.screen_4_label_19 = NULL;
    guider_ui.screen_4_label_24 = NULL;
    guider_ui.screen_4_label_23 = NULL;
    guider_ui.screen_4_label_22 = NULL;
    guider_ui.screen_4_ddlist_4 = NULL;
}

// ---------------------------------------------------------
// Screen 0 事件
// ---------------------------------------------------------
static void screen_btn_1_event_handler (lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    switch (code) {
    case LV_EVENT_CLICKED:
    {
        /* 1. 先彻底斩断旧界面的所有指针联系 */
        clear_all_old_pointers();
        /* 2. 再加载新界面（此时会重新为新界面上的控件分配有效地址） */
        ui_load_scr_animation(&guider_ui, &guider_ui.screen_1, guider_ui.screen_1_del, &guider_ui.screen_del, setup_scr_screen_1, LV_SCR_LOAD_ANIM_NONE, 200, 200, true, true);
        break;
    }
    default:
        break;
    }
}

void events_init_screen (lv_ui *ui)
{
    lv_obj_add_event_cb(ui->screen_btn_1, screen_btn_1_event_handler, LV_EVENT_ALL, ui);
}

// ---------------------------------------------------------
// Screen 1 事件
// ---------------------------------------------------------
static void screen_1_event_handler (lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    switch (code) {
    case LV_EVENT_CLICKED:
    {
        /* 强制标记所有 UI 需刷新 */
        g_ui_data.update_flags = 0xFFFFFFFF;
        clear_all_old_pointers();
        ui_load_scr_animation(&guider_ui, &guider_ui.screen, guider_ui.screen_del, &guider_ui.screen_1_del, setup_scr_screen, LV_SCR_LOAD_ANIM_NONE, 200, 200, true, true);
        break;
    }
    default:
        break;
    }
}

static void screen_1_btn_1_event_handler (lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    switch (code) {
    case LV_EVENT_CLICKED:
    {
        /* 强制标记所有 UI 需刷新（解决进入记录页后不显示旧数据的问题） */
        g_ui_data.update_flags = 0xFFFFFFFF;
        clear_all_old_pointers();
        ui_load_scr_animation(&guider_ui, &guider_ui.screen_2, guider_ui.screen_2_del, &guider_ui.screen_1_del, setup_scr_screen_2, LV_SCR_LOAD_ANIM_NONE, 200, 200, true, true);
        break;
    }
    default:
        break;
    }
}

static void screen_1_btn_2_event_handler (lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    switch (code) {
    case LV_EVENT_CLICKED:
    {
        clear_all_old_pointers();
        ui_load_scr_animation(&guider_ui, &guider_ui.screen_3, guider_ui.screen_3_del, &guider_ui.screen_1_del, setup_scr_screen_3, LV_SCR_LOAD_ANIM_NONE, 200, 200, true, true);
        break;
    }
    default:
        break;
    }
}

static void screen_1_btn_3_event_handler (lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    switch (code) {
    case LV_EVENT_CLICKED:
    {
        /* 强制标记所有 UI 需刷新 */
        g_ui_data.update_flags = 0xFFFFFFFF;
        clear_all_old_pointers();
        ui_load_scr_animation(&guider_ui, &guider_ui.screen_4, guider_ui.screen_4_del, &guider_ui.screen_1_del, setup_scr_screen_4, LV_SCR_LOAD_ANIM_NONE, 200, 200, true, true);
        break;
    }
    default:
        break;
    }
}

static void screen_1_btn_4_event_handler (lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    switch (code) {
    case LV_EVENT_CLICKED:
    {
        /* 强制标记所有 UI 需刷新 */
        g_ui_data.update_flags = 0xFFFFFFFF;
        clear_all_old_pointers();
        ui_load_scr_animation(&guider_ui, &guider_ui.screen, guider_ui.screen_del, &guider_ui.screen_1_del, setup_scr_screen, LV_SCR_LOAD_ANIM_NONE, 200, 200, true, true);
        break;
    }
    default:
        break;
    }
}

void events_init_screen_1 (lv_ui *ui)
{
    lv_obj_add_event_cb(ui->screen_1, screen_1_event_handler, LV_EVENT_ALL, ui);
    lv_obj_add_event_cb(ui->screen_1_btn_1, screen_1_btn_1_event_handler, LV_EVENT_ALL, ui);
    lv_obj_add_event_cb(ui->screen_1_btn_2, screen_1_btn_2_event_handler, LV_EVENT_ALL, ui);
    lv_obj_add_event_cb(ui->screen_1_btn_3, screen_1_btn_3_event_handler, LV_EVENT_ALL, ui);
    lv_obj_add_event_cb(ui->screen_1_btn_4, screen_1_btn_4_event_handler, LV_EVENT_ALL, ui);
}

// ---------------------------------------------------------
// Screen 2 事件
// ---------------------------------------------------------
static void screen_2_btn_1_event_handler (lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    switch (code) {
    case LV_EVENT_CLICKED:
    {
        clear_all_old_pointers();
        ui_load_scr_animation(&guider_ui, &guider_ui.screen_1, guider_ui.screen_1_del, &guider_ui.screen_2_del, setup_scr_screen_1, LV_SCR_LOAD_ANIM_NONE, 200, 200, true, true);
        break;
    }
    default:
        break;
    }
}

void events_init_screen_2 (lv_ui *ui)
{
    lv_obj_add_event_cb(ui->screen_2_btn_1, screen_2_btn_1_event_handler, LV_EVENT_ALL, ui);
}

// ---------------------------------------------------------
// Screen 3 事件
// ---------------------------------------------------------
static void screen_3_btn_1_event_handler (lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    switch (code) {
    case LV_EVENT_CLICKED:
    {
        clear_all_old_pointers();
        ui_load_scr_animation(&guider_ui, &guider_ui.screen_1, guider_ui.screen_1_del, &guider_ui.screen_3_del, setup_scr_screen_1, LV_SCR_LOAD_ANIM_NONE, 200, 200, true, true);
        break;
    }
    default:
        break;
    }
}

void events_init_screen_3 (lv_ui *ui)
{
    lv_obj_add_event_cb(ui->screen_3_btn_1, screen_3_btn_1_event_handler, LV_EVENT_ALL, ui);
}

// ---------------------------------------------------------
// Screen 4 事件
// ---------------------------------------------------------
static void screen_4_btn_1_event_handler (lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    switch (code) {
    case LV_EVENT_CLICKED:
    {
        /* =================== 仅保存用药颗数 =================== */
        char buf[4];

        // 药品1的颗数设置: ddlist_4
        lv_dropdown_get_selected_str(guider_ui.screen_4_ddlist_4, buf, sizeof(buf));
        g_ui_data.meds_schedule[0].pill_count = (uint8_t)atoi(buf);

        // 药品2的颗数设置: ddlist_2
        lv_dropdown_get_selected_str(guider_ui.screen_4_ddlist_2, buf, sizeof(buf));
        g_ui_data.meds_schedule[1].pill_count = (uint8_t)atoi(buf);

        // 药品3的颗数设置: ddlist_3
        lv_dropdown_get_selected_str(guider_ui.screen_4_ddlist_3, buf, sizeof(buf));
        g_ui_data.meds_schedule[2].pill_count = (uint8_t)atoi(buf);

        // 同步到全局 my_meds 并保存 Flash (注：hour 和 min 字段在此过程中保持不变)
        memcpy(my_meds, g_ui_data.meds_schedule, sizeof(my_meds));
        SystemData_Save_To_Flash();

        // 更新闹钟
        BSP_RTC_UpdateNextAlarm();
        
        clear_all_old_pointers();
        ui_load_scr_animation(&guider_ui, &guider_ui.screen_1, guider_ui.screen_1_del, &guider_ui.screen_4_del, setup_scr_screen_1, LV_SCR_LOAD_ANIM_NONE, 200, 200, true, true);
        break;
    }

    default:
        break;
    }
}

void events_init_screen_4 (lv_ui *ui)
{
    lv_obj_add_event_cb(ui->screen_4_btn_1, screen_4_btn_1_event_handler, LV_EVENT_ALL, ui);
}


void events_init(lv_ui *ui)
{

}