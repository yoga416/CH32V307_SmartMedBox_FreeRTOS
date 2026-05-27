/*
* Copyright 2026 NXP
* NXP Proprietary. This software is owned or controlled by NXP and may only be used strictly in
* accordance with the applicable license terms. By expressly accepting such terms or by downloading, installing,
* activating and/or otherwise using the software, you are agreeing that you have read, and that you agree to
* comply with and are bound by, such license terms.  If you do not agree to be bound by the applicable license
* terms, then you may not retain, install, activate or otherwise use the software.
*/

#include "lvgl.h"
#include <stdio.h>
#include <stdbool.h>
#include "gui_guider.h"
#include "events_init.h"
#include "widgets_init.h"
#include "custom.h"

// 声明外部检测函数：由其他文件提供
extern bool check_medication_time(void);
extern UI_DisplayData_t g_ui_data;
extern void SystemData_Save_To_Flash(void);

// “吃药”按钮的点击事件回调函数
static void btn_take_med_event_cb(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * btn = lv_event_get_target(e);
    
    if(code == LV_EVENT_CLICKED) {
        // 获取按钮内部的 Label 并修改文字
        lv_obj_t * label = lv_obj_get_child(btn, 0); 
        if(label != NULL) {
            lv_label_set_text(label, "记录"); 
        }
        
        // 禁用该按钮，防止重复点击
        lv_obj_add_state(btn, LV_STATE_DISABLED); 
        
       // 获取触发按钮的指针，并更新对应的状态
    if (btn == guider_ui.screen_3_btn_med1) {
       g_ui_data.med_status[0] = 1; // 记录药品一已服用
    } else if (btn == guider_ui.screen_3_btn_med2) {
        g_ui_data.med_status[1] = 1; // 记录药品二已服用
    } else if (btn == guider_ui.screen_3_btn_med3) {
        g_ui_data.med_status[2] = 1; // 记录药品三已服用
    }
    
    /*保存状态*/
   SystemData_Save_To_Flash();
    }
}


void setup_scr_screen_3(lv_ui *ui)
{
    // Write codes screen_3
    ui->screen_3 = lv_obj_create(NULL);
    lv_obj_set_size(ui->screen_3, 320, 480);
    lv_obj_set_scrollbar_mode(ui->screen_3, LV_SCROLLBAR_MODE_OFF);

    // Write style for screen_3, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_bg_opa(ui->screen_3, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

    // Write codes screen_3_cont_1
    ui->screen_3_cont_1 = lv_obj_create(ui->screen_3);
    lv_obj_set_pos(ui->screen_3_cont_1, 0, 0);
    lv_obj_set_size(ui->screen_3_cont_1, 320, 480);
    lv_obj_set_scrollbar_mode(ui->screen_3_cont_1, LV_SCROLLBAR_MODE_OFF);

    // Write style for screen_3_cont_1
    lv_obj_set_style_border_width(ui->screen_3_cont_1, 2, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui->screen_3_cont_1, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui->screen_3_cont_1, lv_color_hex(0x2195f6), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_side(ui->screen_3_cont_1, LV_BORDER_SIDE_FULL, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_3_cont_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui->screen_3_cont_1, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen_3_cont_1, lv_color_hex(0xffffff), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_3_cont_1, LV_GRAD_DIR_NONE, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_all(ui->screen_3_cont_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_3_cont_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

    // ================== 核心逻辑：界面内容分发 ==================
    if (check_medication_time()) {
        /*
         * 状态一：已到吃药时间
         * 动态生成药品一、药品二、药品三的提示文本及对应确认按钮
         */
        
        // 药品一
        lv_obj_t * label_med1 = lv_label_create(ui->screen_3_cont_1);
        lv_label_set_text(label_med1, "药");
        lv_obj_set_style_text_font(label_med1, &lv_font_SourceHanSerifSC_Regular_20, 0);
        lv_obj_set_pos(label_med1, 40, 80);
        
        lv_obj_t * btn_med1 = lv_btn_create(ui->screen_3_cont_1);
        lv_obj_set_pos(btn_med1, 180, 70);
        lv_obj_set_size(btn_med1, 80, 40);
        lv_obj_t * btn_med1_label = lv_label_create(btn_med1);
        lv_obj_set_style_text_font(btn_med1_label, &lv_font_SourceHanSerifSC_Regular_20, 0);
        lv_obj_center(btn_med1_label);
        // 根据 g_med_status 设置初始状态
        if (g_ui_data.med_status[0]==1) {
            lv_label_set_text(btn_med1_label, "记录");
            lv_obj_add_state(btn_med1, LV_STATE_DISABLED);
        } else {
            lv_label_set_text(btn_med1_label, "吃药");
        }
        lv_obj_add_event_cb(btn_med1, btn_take_med_event_cb, LV_EVENT_CLICKED, NULL);
        ui->screen_3_btn_med1 = btn_med1;

        // 药品二
        lv_obj_t * label_med2 = lv_label_create(ui->screen_3_cont_1);
        lv_label_set_text(label_med2, "药");
        lv_obj_set_style_text_font(label_med2, &lv_font_SourceHanSerifSC_Regular_20, 0);
        lv_obj_set_pos(label_med2, 40, 150);
        
        lv_obj_t * btn_med2 = lv_btn_create(ui->screen_3_cont_1);
        lv_obj_set_pos(btn_med2, 180, 140);
        lv_obj_set_size(btn_med2, 80, 40);
        lv_obj_t * btn_med2_label = lv_label_create(btn_med2);
        lv_obj_set_style_text_font(btn_med2_label, &lv_font_SourceHanSerifSC_Regular_20, 0);
        lv_obj_center(btn_med2_label);
        // 根据 g_ui_data.med_status 设置初始状态
        if (g_ui_data.med_status[1]==1) {
            lv_label_set_text(btn_med2_label, "记录");
            lv_obj_add_state(btn_med2, LV_STATE_DISABLED);
        } else {
            lv_label_set_text(btn_med2_label, "吃药");
        }
        lv_obj_add_event_cb(btn_med2, btn_take_med_event_cb, LV_EVENT_CLICKED, NULL);
        ui->screen_3_btn_med2 = btn_med2; // 【修复】保存指针到 guider_ui

        // 药品三
        lv_obj_t * label_med3 = lv_label_create(ui->screen_3_cont_1);
        lv_label_set_text(label_med3, "药");
        lv_obj_set_style_text_font(label_med3, &lv_font_SourceHanSerifSC_Regular_20, 0);
        lv_obj_set_pos(label_med3, 40, 220);
        
        lv_obj_t * btn_med3 = lv_btn_create(ui->screen_3_cont_1);
        lv_obj_set_pos(btn_med3, 180, 210);
        lv_obj_set_size(btn_med3, 80, 40);
        lv_obj_t * btn_med3_label = lv_label_create(btn_med3);
        lv_obj_set_style_text_font(btn_med3_label, &lv_font_SourceHanSerifSC_Regular_20, 0);
        lv_obj_center(btn_med3_label);
        // 根据 g_ui_data.med_status 设置初始状态
        if (g_ui_data.med_status[2]==1) {
            lv_label_set_text(btn_med3_label, "记录");
            lv_obj_add_state(btn_med3, LV_STATE_DISABLED);
        } else {
            lv_label_set_text(btn_med3_label, "吃药");
        }
        lv_obj_add_event_cb(btn_med3, btn_take_med_event_cb, LV_EVENT_CLICKED, NULL);
        ui->screen_3_btn_med3 = btn_med3; // 【修复】保存指针到 guider_ui

    } else {
        /*
         * 状态二：未到吃药时间
         * 仅显示文本提示
         */
        ui->screen_3_label_1 = lv_label_create(ui->screen_3_cont_1);
        lv_label_set_text(ui->screen_3_label_1, "不在用药时间哦");
        lv_label_set_long_mode(ui->screen_3_label_1, LV_LABEL_LONG_WRAP);
        lv_obj_set_pos(ui->screen_3_label_1, 43, 175);
        lv_obj_set_size(ui->screen_3_label_1, 211, 43);

        lv_obj_set_style_border_width(ui->screen_3_label_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
        lv_obj_set_style_radius(ui->screen_3_label_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
        lv_obj_set_style_text_color(ui->screen_3_label_1, lv_color_hex(0x000000), LV_PART_MAIN|LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(ui->screen_3_label_1, &lv_font_SourceHanSerifSC_Regular_20, LV_PART_MAIN|LV_STATE_DEFAULT);
        lv_obj_set_style_text_opa(ui->screen_3_label_1, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
        lv_obj_set_style_text_letter_space(ui->screen_3_label_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
        lv_obj_set_style_text_line_space(ui->screen_3_label_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
        lv_obj_set_style_text_align(ui->screen_3_label_1, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN|LV_STATE_DEFAULT);
        lv_obj_set_style_bg_opa(ui->screen_3_label_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
        lv_obj_set_style_pad_all(ui->screen_3_label_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_width(ui->screen_3_label_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    }
    // ==========================================================

    // Write codes screen_3_btn_1 (下方公共的 Back 返回按钮)
    ui->screen_3_btn_1 = lv_btn_create(ui->screen_3_cont_1);
    ui->screen_3_btn_1_label = lv_label_create(ui->screen_3_btn_1);
    lv_label_set_text(ui->screen_3_btn_1_label, "back");
    lv_label_set_long_mode(ui->screen_3_btn_1_label, LV_LABEL_LONG_WRAP);
    lv_obj_align(ui->screen_3_btn_1_label, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_pad_all(ui->screen_3_btn_1, 0, LV_STATE_DEFAULT);
    lv_obj_set_width(ui->screen_3_btn_1_label, LV_PCT(100));
    
    // 居中靠下放置返回按钮，避免与吃药列表遮挡
    lv_obj_set_pos(ui->screen_3_btn_1, 113, 350); 
    lv_obj_set_size(ui->screen_3_btn_1, 94, 45);

    // Write style for screen_3_btn_1
    lv_obj_set_style_bg_opa(ui->screen_3_btn_1, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen_3_btn_1, lv_color_hex(0x2195f6), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_3_btn_1, LV_GRAD_DIR_NONE, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui->screen_3_btn_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_3_btn_1, 5, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_3_btn_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui->screen_3_btn_1, lv_color_hex(0xffffff), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->screen_3_btn_1, &lv_font_montserratMedium_20, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->screen_3_btn_1, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui->screen_3_btn_1, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN|LV_STATE_DEFAULT);

    // Update current screen layout.
    lv_obj_update_layout(ui->screen_3);

    // Init events for screen.
    events_init_screen_3(ui);
}