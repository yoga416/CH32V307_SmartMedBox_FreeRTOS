/*
 * Copyright 2023 NXP
 * NXP Proprietary. This software is owned or controlled by NXP and may only be used strictly in
 * accordance with the applicable license terms. By expressly accepting such terms or by downloading, installing,
 * activating and/or otherwise using the software, you are agreeing that you have read, and that you agree to
 * comply with and are bound by, such license terms.  If you do not agree to be bound by the applicable license
 * terms, then you may not retain, install, activate or otherwise use the software.
 */

/*********************
 * INCLUDES
 *********************/
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "lvgl.h"
#include "custom.h"
#include "gui_guider.h"
#include "information.h" 
#include "app_task.h"    // UI_FLAG_MED_STATUS 等标志定义

/*********************
 * DEFINES
 *********************/

/**********************
 * TYPEDEFS
 **********************/
/* GUI Guider 自动生成的 UI 结构体，里面包含所有界面的控件指针 */
extern lv_ui guider_ui;

/**********************
 * STATIC VARIABLES
 **********************/
/* 这些变量用于弹窗的显示与控制 */
static lv_obj_t * time_popup = NULL;   // 弹窗的背景遮罩层
static lv_obj_t * roller_h = NULL;     // 小时选择滚轮
static lv_obj_t * roller_m = NULL;     // 分钟选择滚轮
static uint8_t current_edit_idx = 0;   // 记录当前修改的是哪一顿药：0=第一顿，1=第二顿，2=第三顿

/* 滚轮的选项字符串缓冲 */
static char hour_opts[100] = {0};
static char min_opts[200] = {0};

/**********************
 * STATIC PROTOTYPES & FUNCTIONS
 **********************/

/* 确认按钮回调函数 */
static void btn_ok_cb(lv_event_t * e)
{
    // 1. 获取用户在滚轮上选择的小时和分钟
    uint16_t h = lv_roller_get_selected(roller_h);
    uint16_t m = lv_roller_get_selected(roller_m);

    // 2. 更新当前活跃用户的数据到全局数组 g_ui_data 中
    g_ui_data[g_current_active_user_id].meds_schedule[current_edit_idx].hour = (uint8_t)h;
    g_ui_data[g_current_active_user_id].meds_schedule[current_edit_idx].min = (uint8_t)m;

    // 3. 核心步骤：调用 information.c 中的函数，将修改后的数据存入 W25Qxx Flash 中！
    SystemData_Save_To_Flash_ByUser(g_current_active_user_id);

    // 4. 触发 UI 刷新（仅本地更新 Label，不触发上传）
    g_ui_data[g_current_active_user_id].update_flags |= UI_FLAG_SCHEDULE;

    // 5. 销毁弹窗，释放内存
    lv_obj_del(time_popup);
    time_popup = NULL;
}

/* 取消按钮回调函数 */
static void btn_cancel_cb(lv_event_t * e)
{
    // 什么都不存，直接销毁弹窗
    lv_obj_del(time_popup);
    time_popup = NULL;
}

/* 吃药时间 Label 点击事件：生成时间选择弹窗 */
static void time_label_click_cb(lv_event_t * e)
{
    // 如果弹窗已经打开，防止用户狂点重复创建
    if(time_popup != NULL) return;

    // 获取我们在 custom_init 里绑定时传过来的“用户数据 (User Data)”
    current_edit_idx = (uint32_t)(uintptr_t)lv_event_get_user_data(e);

    // 从 Flash 加载的数据中获取该顿药的当前时间，作为滚轮的默认初始值
    uint8_t cur_h = g_ui_data[g_current_active_user_id].meds_schedule[current_edit_idx].hour;
    uint8_t cur_m = g_ui_data[g_current_active_user_id].meds_schedule[current_edit_idx].min;

    // --- 开始画 UI：创建全屏半透明遮罩背景 ---
    time_popup = lv_obj_create(lv_scr_act()); 
    lv_obj_set_size(time_popup, LV_PCT(100), LV_PCT(100)); 
    lv_obj_set_style_bg_color(time_popup, lv_color_hex(0x000000), 0); 
    lv_obj_set_style_bg_opa(time_popup, LV_OPA_50, 0); 
    lv_obj_set_style_border_width(time_popup, 0, 0); 

    // --- 创建居中的小面板 ---
    lv_obj_t * panel = lv_obj_create(time_popup);
    lv_obj_set_size(panel, 240, 180); 
    lv_obj_center(panel); 

    // --- 生成滚轮需要的字符串 ("00\n01\n02...\n23") ---
    if(hour_opts[0] == '\0') {
        for(int i = 0; i < 24; i++) {
            char buf[5]; sprintf(buf, "%02d\n", i); strcat(hour_opts, buf);
        }
        hour_opts[strlen(hour_opts)-1] = '\0'; 
        for(int i = 0; i < 60; i++) {
            char buf[5]; sprintf(buf, "%02d\n", i); strcat(min_opts, buf);
        }
        min_opts[strlen(min_opts)-1] = '\0';
    }

    // --- 创建小时选择滚轮 (Roller) ---
    roller_h = lv_roller_create(panel);
    lv_roller_set_options(roller_h, hour_opts, LV_ROLLER_MODE_NORMAL); 
    lv_roller_set_selected(roller_h, cur_h, LV_ANIM_OFF); 
    lv_roller_set_visible_row_count(roller_h, 3); 
    lv_obj_align(roller_h, LV_ALIGN_LEFT_MID, 10, -20); 

    // --- 创建中间的冒号 ":" ---
    lv_obj_t * colon = lv_label_create(panel);
    lv_label_set_text(colon, ":");
    lv_obj_center(colon);
    lv_obj_align(colon, LV_ALIGN_CENTER, 0, -20);

    // --- 创建分钟选择滚轮 (Roller) ---
    roller_m = lv_roller_create(panel);
    lv_roller_set_options(roller_m, min_opts, LV_ROLLER_MODE_NORMAL);
    lv_roller_set_selected(roller_m, cur_m, LV_ANIM_OFF);
    lv_roller_set_visible_row_count(roller_m, 3);
    lv_obj_align(roller_m, LV_ALIGN_RIGHT_MID, -10, -20); 

    // --- 创建 OK 确认按钮 ---
    lv_obj_t * btn_ok = lv_btn_create(panel);
    lv_obj_set_size(btn_ok, 80, 40);
    lv_obj_align(btn_ok, LV_ALIGN_BOTTOM_LEFT, 10, 10);
    lv_obj_add_event_cb(btn_ok, btn_ok_cb, LV_EVENT_CLICKED, NULL); 
    lv_obj_t * label_ok = lv_label_create(btn_ok);
    lv_label_set_text(label_ok, "OK");
    lv_obj_center(label_ok);

    // --- 创建 Cancel 取消按钮 ---
    lv_obj_t * btn_cancel = lv_btn_create(panel);
    lv_obj_set_size(btn_cancel, 80, 40);
    lv_obj_align(btn_cancel, LV_ALIGN_BOTTOM_RIGHT, -10, 10);
    lv_obj_add_event_cb(btn_cancel, btn_cancel_cb, LV_EVENT_CLICKED, NULL); 
    lv_obj_t * label_cancel = lv_label_create(btn_cancel);
    lv_label_set_text(label_cancel, "Cancel");
    lv_obj_center(label_cancel);
}

/* 将原本 custom_init 里的绑定代码移出来，做成一个独立的动态绑定函数 */
void custom_bind_time_labels(void)
{
    // 如果控件已经生成，且还没被设置为"可点击"，则为它绑定事件
    if (lv_obj_is_valid(guider_ui.screen_4_label_2)) {
        if (!lv_obj_has_flag(guider_ui.screen_4_label_2, LV_OBJ_FLAG_CLICKABLE)) {
            lv_obj_add_flag(guider_ui.screen_4_label_2, LV_OBJ_FLAG_CLICKABLE);
            lv_obj_add_event_cb(guider_ui.screen_4_label_2, time_label_click_cb, LV_EVENT_CLICKED, (void*)(uintptr_t)0); 
        }
    }
    
    if (lv_obj_is_valid(guider_ui.screen_4_label_20)) {
        if (!lv_obj_has_flag(guider_ui.screen_4_label_20, LV_OBJ_FLAG_CLICKABLE)) {
            lv_obj_add_flag(guider_ui.screen_4_label_20, LV_OBJ_FLAG_CLICKABLE);
            lv_obj_add_event_cb(guider_ui.screen_4_label_20, time_label_click_cb, LV_EVENT_CLICKED, (void*)(uintptr_t)1);
        }
    }

    if (lv_obj_is_valid(guider_ui.screen_4_label_23)) {
        if (!lv_obj_has_flag(guider_ui.screen_4_label_23, LV_OBJ_FLAG_CLICKABLE)) {
            lv_obj_add_flag(guider_ui.screen_4_label_23, LV_OBJ_FLAG_CLICKABLE);
            lv_obj_add_event_cb(guider_ui.screen_4_label_23, time_label_click_cb, LV_EVENT_CLICKED, (void*)(uintptr_t)2);
        }
    }
}
/**
 * Create a demo application
 * GUI Guider 在创建完组件后会调用这个函数，我们将绑定逻辑放在这里
 */
void custom_init(lv_ui *ui)
{
    // 1. 绑定吃药时间标签的点击事件，生成时间选择弹窗

    // 2. 其他初始化代码（如果有）可以放在这里
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

/* ========== 从 setup_scr_screen_3.c 迁移过来的吃药按钮回调 ========== */
/* “吃药”按钮的点击事件回调函数 */
void btn_take_med_event_cb(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * btn = lv_event_get_target(e);
    if(code == LV_EVENT_CLICKED) {
        // 获取按钮内部的 Label 并修改文字
        lv_obj_t * label = lv_obj_get_child(btn, 0); 
        if(label != NULL) {
            lv_label_set_text(label, "记录"); 
        }
        // 获取触发按钮的指针，并更新对应的状态
        if (btn == guider_ui.screen_3_btn_med1) {
            g_ui_data[g_current_active_user_id].med_status[0] = true; // 记录药品一已服用
        } else if (btn == guider_ui.screen_3_btn_med2) {
            g_ui_data[g_current_active_user_id].med_status[1] = true; // 记录药品二已服用
        } else if (btn == guider_ui.screen_3_btn_med3) {
            g_ui_data[g_current_active_user_id].med_status[2] = true; // 记录药品三已服用
        }
        // 三顿药都吃完则标记 meds_completed
        if(g_ui_data[g_current_active_user_id].med_status[0]==1 && 
           g_ui_data[g_current_active_user_id].med_status[1]==1 && 
           g_ui_data[g_current_active_user_id].med_status[2]==1) {
            // 查找当前哪一顿还未完成
            for(int i = 0; i < 3; i++) {
                if(g_ui_data[g_current_active_user_id].meds_completed[i] == 0) {
                    g_ui_data[g_current_active_user_id].meds_completed[i] = 1;
                    break;
                }
            }
        }
        // 禁用该按钮，防止重复点击
        lv_obj_add_state(btn, LV_STATE_DISABLED); 
        g_ui_data[g_current_active_user_id].update_flags |= UI_FLAG_MED_STATUS; // 触发 UI 刷新
        /*保存状态*/
        SystemData_Save_To_Flash_ByUser(g_current_active_user_id);
    }
}

void gui_protect_pointer_on_delete(lv_obj_t *obj, lv_obj_t **pptr)
{
    if (obj != NULL && pptr != NULL) {
        lv_obj_add_event_cb(obj, gui_label_delete_cb, LV_EVENT_DELETE, pptr);
    }
}