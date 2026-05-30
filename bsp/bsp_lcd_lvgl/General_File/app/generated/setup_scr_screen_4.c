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
#include "gui_guider.h"
#include "events_init.h"
#include "widgets_init.h"
#include "custom.h"



void setup_scr_screen_4(lv_ui *ui)
{
    //Write codes screen_4
    ui->screen_4 = lv_obj_create(NULL);
    lv_obj_set_size(ui->screen_4, 320, 480);
    lv_obj_set_scrollbar_mode(ui->screen_4, LV_SCROLLBAR_MODE_OFF);

    //Write style for screen_4, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_bg_opa(ui->screen_4, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write codes screen_4_cont_1
    ui->screen_4_cont_1 = lv_obj_create(ui->screen_4);
    lv_obj_set_pos(ui->screen_4_cont_1, 0, 0);
    lv_obj_set_size(ui->screen_4_cont_1, 320, 480);
    lv_obj_set_scrollbar_mode(ui->screen_4_cont_1, LV_SCROLLBAR_MODE_OFF);

    //Write style for screen_4_cont_1, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_border_width(ui->screen_4_cont_1, 2, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui->screen_4_cont_1, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui->screen_4_cont_1, lv_color_hex(0x2195f6), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_side(ui->screen_4_cont_1, LV_BORDER_SIDE_FULL, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_4_cont_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui->screen_4_cont_1, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen_4_cont_1, lv_color_hex(0xffffff), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_4_cont_1, LV_GRAD_DIR_NONE, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui->screen_4_cont_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui->screen_4_cont_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui->screen_4_cont_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui->screen_4_cont_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_4_cont_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write codes screen_4_label_1
    ui->screen_4_label_1 = lv_label_create(ui->screen_4_cont_1);
    lv_label_set_text(ui->screen_4_label_1, "Locating:1");
    lv_label_set_long_mode(ui->screen_4_label_1, LV_LABEL_LONG_WRAP);
    lv_obj_set_pos(ui->screen_4_label_1, 10, 23);
    lv_obj_set_size(ui->screen_4_label_1, 135, 27);

    //Write style for screen_4_label_1, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_border_width(ui->screen_4_label_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_4_label_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui->screen_4_label_1, lv_color_hex(0x000000), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->screen_4_label_1, &lv_font_montserrat_16, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->screen_4_label_1, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(ui->screen_4_label_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_line_space(ui->screen_4_label_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui->screen_4_label_1, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui->screen_4_label_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui->screen_4_label_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui->screen_4_label_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui->screen_4_label_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui->screen_4_label_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_4_label_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write codes screen_4_label_2
    ui->screen_4_label_2 = lv_label_create(ui->screen_4_cont_1);
    lv_label_set_text(ui->screen_4_label_2, "Time:--");
    lv_label_set_long_mode(ui->screen_4_label_2, LV_LABEL_LONG_WRAP);
    lv_obj_set_pos(ui->screen_4_label_2, -60, 55);
    lv_obj_set_size(ui->screen_4_label_2, 320, 27);

    //Write style for screen_4_label_2, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_border_width(ui->screen_4_label_2, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_4_label_2, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui->screen_4_label_2, lv_color_hex(0x000000), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->screen_4_label_2, &lv_font_montserrat_16, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->screen_4_label_2, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(ui->screen_4_label_2, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_line_space(ui->screen_4_label_2, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui->screen_4_label_2, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui->screen_4_label_2, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui->screen_4_label_2, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui->screen_4_label_2, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui->screen_4_label_2, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui->screen_4_label_2, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_4_label_2, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write codes screen_4_label_3
    ui->screen_4_label_3 = lv_label_create(ui->screen_4_cont_1);
    lv_label_set_text(ui->screen_4_label_3, "Number:");
    lv_label_set_long_mode(ui->screen_4_label_3, LV_LABEL_LONG_WRAP);
    lv_obj_set_pos(ui->screen_4_label_3, 15, 89);
    lv_obj_set_size(ui->screen_4_label_3, 113, 27);

    //Write style for screen_4_label_3, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_border_width(ui->screen_4_label_3, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_4_label_3, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui->screen_4_label_3, lv_color_hex(0x000000), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->screen_4_label_3, &lv_font_montserrat_16, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->screen_4_label_3, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(ui->screen_4_label_3, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_line_space(ui->screen_4_label_3, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui->screen_4_label_3, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui->screen_4_label_3, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui->screen_4_label_3, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui->screen_4_label_3, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui->screen_4_label_3, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui->screen_4_label_3, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_4_label_3, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write codes screen_4_ddlist_2
    ui->screen_4_ddlist_2 = lv_dropdown_create(ui->screen_4_cont_1);
    lv_dropdown_set_options(ui->screen_4_ddlist_2, "1\n2\n3\n4\n5");
    lv_obj_set_pos(ui->screen_4_ddlist_2, 152, 198);
    lv_obj_set_size(ui->screen_4_ddlist_2, 130, 30);

    //Write style for screen_4_ddlist_2, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_text_color(ui->screen_4_ddlist_2, lv_color_hex(0x0D3055), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->screen_4_ddlist_2, &lv_font_montserrat_16, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->screen_4_ddlist_2, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui->screen_4_ddlist_2, 1, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui->screen_4_ddlist_2, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui->screen_4_ddlist_2, lv_color_hex(0xe1e6ee), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_side(ui->screen_4_ddlist_2, LV_BORDER_SIDE_FULL, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui->screen_4_ddlist_2, 8, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui->screen_4_ddlist_2, 6, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui->screen_4_ddlist_2, 6, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_4_ddlist_2, 3, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui->screen_4_ddlist_2, 28, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen_4_ddlist_2, lv_color_hex(0x000000), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_4_ddlist_2, LV_GRAD_DIR_NONE, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_4_ddlist_2, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write style state: LV_STATE_CHECKED for &style_screen_4_ddlist_2_extra_list_selected_checked
    static lv_style_t style_screen_4_ddlist_2_extra_list_selected_checked;
    ui_init_style(&style_screen_4_ddlist_2_extra_list_selected_checked);

    lv_style_set_border_width(&style_screen_4_ddlist_2_extra_list_selected_checked, 1);
    lv_style_set_border_opa(&style_screen_4_ddlist_2_extra_list_selected_checked, 255);
    lv_style_set_border_color(&style_screen_4_ddlist_2_extra_list_selected_checked, lv_color_hex(0xe1e6ee));
    lv_style_set_border_side(&style_screen_4_ddlist_2_extra_list_selected_checked, LV_BORDER_SIDE_FULL);
    lv_style_set_radius(&style_screen_4_ddlist_2_extra_list_selected_checked, 3);
    lv_style_set_bg_opa(&style_screen_4_ddlist_2_extra_list_selected_checked, 255);
    lv_style_set_bg_color(&style_screen_4_ddlist_2_extra_list_selected_checked, lv_color_hex(0x00a1b5));
    lv_style_set_bg_grad_dir(&style_screen_4_ddlist_2_extra_list_selected_checked, LV_GRAD_DIR_NONE);
    lv_obj_add_style(lv_dropdown_get_list(ui->screen_4_ddlist_2), &style_screen_4_ddlist_2_extra_list_selected_checked, LV_PART_SELECTED|LV_STATE_CHECKED);

    //Write style state: LV_STATE_DEFAULT for &style_screen_4_ddlist_2_extra_list_main_default
    static lv_style_t style_screen_4_ddlist_2_extra_list_main_default;
    ui_init_style(&style_screen_4_ddlist_2_extra_list_main_default);

    lv_style_set_max_height(&style_screen_4_ddlist_2_extra_list_main_default, 90);
    lv_style_set_text_color(&style_screen_4_ddlist_2_extra_list_main_default, lv_color_hex(0x0D3055));
    lv_style_set_text_font(&style_screen_4_ddlist_2_extra_list_main_default, &lv_font_montserratMedium_12);
    lv_style_set_text_opa(&style_screen_4_ddlist_2_extra_list_main_default, 255);
    lv_style_set_border_width(&style_screen_4_ddlist_2_extra_list_main_default, 1);
    lv_style_set_border_opa(&style_screen_4_ddlist_2_extra_list_main_default, 255);
    lv_style_set_border_color(&style_screen_4_ddlist_2_extra_list_main_default, lv_color_hex(0xe1e6ee));
    lv_style_set_border_side(&style_screen_4_ddlist_2_extra_list_main_default, LV_BORDER_SIDE_FULL);
    lv_style_set_radius(&style_screen_4_ddlist_2_extra_list_main_default, 3);
    lv_style_set_bg_opa(&style_screen_4_ddlist_2_extra_list_main_default, 255);
    lv_style_set_bg_color(&style_screen_4_ddlist_2_extra_list_main_default, lv_color_hex(0xffffff));
    lv_style_set_bg_grad_dir(&style_screen_4_ddlist_2_extra_list_main_default, LV_GRAD_DIR_NONE);
    lv_obj_add_style(lv_dropdown_get_list(ui->screen_4_ddlist_2), &style_screen_4_ddlist_2_extra_list_main_default, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write style state: LV_STATE_DEFAULT for &style_screen_4_ddlist_2_extra_list_scrollbar_default
    static lv_style_t style_screen_4_ddlist_2_extra_list_scrollbar_default;
    ui_init_style(&style_screen_4_ddlist_2_extra_list_scrollbar_default);

    lv_style_set_radius(&style_screen_4_ddlist_2_extra_list_scrollbar_default, 3);
    lv_style_set_bg_opa(&style_screen_4_ddlist_2_extra_list_scrollbar_default, 255);
    lv_style_set_bg_color(&style_screen_4_ddlist_2_extra_list_scrollbar_default, lv_color_hex(0x00ff00));
    lv_style_set_bg_grad_dir(&style_screen_4_ddlist_2_extra_list_scrollbar_default, LV_GRAD_DIR_NONE);
    lv_obj_add_style(lv_dropdown_get_list(ui->screen_4_ddlist_2), &style_screen_4_ddlist_2_extra_list_scrollbar_default, LV_PART_SCROLLBAR|LV_STATE_DEFAULT);

    //Write codes screen_4_ddlist_3
    ui->screen_4_ddlist_3 = lv_dropdown_create(ui->screen_4_cont_1);
    lv_dropdown_set_options(ui->screen_4_ddlist_3, "1\n2\n3\n4\n5");
    lv_obj_set_pos(ui->screen_4_ddlist_3, 150, 323);
    lv_obj_set_size(ui->screen_4_ddlist_3, 130, 30);

    //Write style for screen_4_ddlist_3, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_text_color(ui->screen_4_ddlist_3, lv_color_hex(0x0D3055), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->screen_4_ddlist_3, &lv_font_montserrat_16, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->screen_4_ddlist_3, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui->screen_4_ddlist_3, 1, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui->screen_4_ddlist_3, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui->screen_4_ddlist_3, lv_color_hex(0xe1e6ee), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_side(ui->screen_4_ddlist_3, LV_BORDER_SIDE_FULL, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui->screen_4_ddlist_3, 8, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui->screen_4_ddlist_3, 6, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui->screen_4_ddlist_3, 6, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_4_ddlist_3, 3, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui->screen_4_ddlist_3, 28, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen_4_ddlist_3, lv_color_hex(0x000000), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_4_ddlist_3, LV_GRAD_DIR_NONE, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_4_ddlist_3, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write style state: LV_STATE_CHECKED for &style_screen_4_ddlist_3_extra_list_selected_checked
    static lv_style_t style_screen_4_ddlist_3_extra_list_selected_checked;
    ui_init_style(&style_screen_4_ddlist_3_extra_list_selected_checked);

    lv_style_set_border_width(&style_screen_4_ddlist_3_extra_list_selected_checked, 1);
    lv_style_set_border_opa(&style_screen_4_ddlist_3_extra_list_selected_checked, 255);
    lv_style_set_border_color(&style_screen_4_ddlist_3_extra_list_selected_checked, lv_color_hex(0xe1e6ee));
    lv_style_set_border_side(&style_screen_4_ddlist_3_extra_list_selected_checked, LV_BORDER_SIDE_FULL);
    lv_style_set_radius(&style_screen_4_ddlist_3_extra_list_selected_checked, 3);
    lv_style_set_bg_opa(&style_screen_4_ddlist_3_extra_list_selected_checked, 255);
    lv_style_set_bg_color(&style_screen_4_ddlist_3_extra_list_selected_checked, lv_color_hex(0x00a1b5));
    lv_style_set_bg_grad_dir(&style_screen_4_ddlist_3_extra_list_selected_checked, LV_GRAD_DIR_NONE);
    lv_obj_add_style(lv_dropdown_get_list(ui->screen_4_ddlist_3), &style_screen_4_ddlist_3_extra_list_selected_checked, LV_PART_SELECTED|LV_STATE_CHECKED);

    //Write style state: LV_STATE_DEFAULT for &style_screen_4_ddlist_3_extra_list_main_default
    static lv_style_t style_screen_4_ddlist_3_extra_list_main_default;
    ui_init_style(&style_screen_4_ddlist_3_extra_list_main_default);

    lv_style_set_max_height(&style_screen_4_ddlist_3_extra_list_main_default, 90);
    lv_style_set_text_color(&style_screen_4_ddlist_3_extra_list_main_default, lv_color_hex(0x0D3055));
    lv_style_set_text_font(&style_screen_4_ddlist_3_extra_list_main_default, &lv_font_montserrat_16);
    lv_style_set_text_opa(&style_screen_4_ddlist_3_extra_list_main_default, 255);
    lv_style_set_border_width(&style_screen_4_ddlist_3_extra_list_main_default, 1);
    lv_style_set_border_opa(&style_screen_4_ddlist_3_extra_list_main_default, 255);
    lv_style_set_border_color(&style_screen_4_ddlist_3_extra_list_main_default, lv_color_hex(0xe1e6ee));
    lv_style_set_border_side(&style_screen_4_ddlist_3_extra_list_main_default, LV_BORDER_SIDE_FULL);
    lv_style_set_radius(&style_screen_4_ddlist_3_extra_list_main_default, 3);
    lv_style_set_bg_opa(&style_screen_4_ddlist_3_extra_list_main_default, 255);
    lv_style_set_bg_color(&style_screen_4_ddlist_3_extra_list_main_default, lv_color_hex(0xffffff));
    lv_style_set_bg_grad_dir(&style_screen_4_ddlist_3_extra_list_main_default, LV_GRAD_DIR_NONE);
    lv_obj_add_style(lv_dropdown_get_list(ui->screen_4_ddlist_3), &style_screen_4_ddlist_3_extra_list_main_default, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write style state: LV_STATE_DEFAULT for &style_screen_4_ddlist_3_extra_list_scrollbar_default
    static lv_style_t style_screen_4_ddlist_3_extra_list_scrollbar_default;
    ui_init_style(&style_screen_4_ddlist_3_extra_list_scrollbar_default);

    lv_style_set_radius(&style_screen_4_ddlist_3_extra_list_scrollbar_default, 3);
    lv_style_set_bg_opa(&style_screen_4_ddlist_3_extra_list_scrollbar_default, 255);
    lv_style_set_bg_color(&style_screen_4_ddlist_3_extra_list_scrollbar_default, lv_color_hex(0x00ff00));
    lv_style_set_bg_grad_dir(&style_screen_4_ddlist_3_extra_list_scrollbar_default, LV_GRAD_DIR_NONE);
    lv_obj_add_style(lv_dropdown_get_list(ui->screen_4_ddlist_3), &style_screen_4_ddlist_3_extra_list_scrollbar_default, LV_PART_SCROLLBAR|LV_STATE_DEFAULT);

    //Write codes screen_4_btn_1
    ui->screen_4_btn_1 = lv_btn_create(ui->screen_4_cont_1);
    ui->screen_4_btn_1_label = lv_label_create(ui->screen_4_btn_1);
    lv_label_set_text(ui->screen_4_btn_1_label, "back");
    lv_label_set_long_mode(ui->screen_4_btn_1_label, LV_LABEL_LONG_WRAP);
    lv_obj_align(ui->screen_4_btn_1_label, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_pad_all(ui->screen_4_btn_1, 0, LV_STATE_DEFAULT);
    lv_obj_set_width(ui->screen_4_btn_1_label, LV_PCT(100));
    lv_obj_set_pos(ui->screen_4_btn_1, 205, 418);
    lv_obj_set_size(ui->screen_4_btn_1, 94, 45);

    //Write style for screen_4_btn_1, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_bg_opa(ui->screen_4_btn_1, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen_4_btn_1, lv_color_hex(0x2195f6), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_4_btn_1, LV_GRAD_DIR_NONE, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui->screen_4_btn_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_4_btn_1, 5, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_4_btn_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui->screen_4_btn_1, lv_color_hex(0xffffff), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->screen_4_btn_1, &lv_font_montserratMedium_20, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->screen_4_btn_1, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui->screen_4_btn_1, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write codes screen_4_label_21
    ui->screen_4_label_21 = lv_label_create(ui->screen_4_cont_1);
    lv_label_set_text(ui->screen_4_label_21, "Number:");
    lv_label_set_long_mode(ui->screen_4_label_21, LV_LABEL_LONG_WRAP);
    lv_obj_set_pos(ui->screen_4_label_21, 15, 204);
    lv_obj_set_size(ui->screen_4_label_21, 113, 27);

    //Write style for screen_4_label_21, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_border_width(ui->screen_4_label_21, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_4_label_21, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui->screen_4_label_21, lv_color_hex(0x000000), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->screen_4_label_21, &lv_font_montserrat_16, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->screen_4_label_21, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(ui->screen_4_label_21, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_line_space(ui->screen_4_label_21, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui->screen_4_label_21, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui->screen_4_label_21, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui->screen_4_label_21, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui->screen_4_label_21, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui->screen_4_label_21, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui->screen_4_label_21, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_4_label_21, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write codes screen_4_label_20
    ui->screen_4_label_20 = lv_label_create(ui->screen_4_cont_1);
    lv_label_set_text(ui->screen_4_label_20, "Time:--");
    lv_label_set_long_mode(ui->screen_4_label_20, LV_LABEL_LONG_WRAP);
    lv_obj_set_pos(ui->screen_4_label_20, -60, 172);
    lv_obj_set_size(ui->screen_4_label_20, 320, 27);

    //Write style for screen_4_label_20, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_border_width(ui->screen_4_label_20, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_4_label_20, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui->screen_4_label_20, lv_color_hex(0x000000), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->screen_4_label_20, &lv_font_montserrat_16, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->screen_4_label_20, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(ui->screen_4_label_20, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_line_space(ui->screen_4_label_20, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui->screen_4_label_20, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui->screen_4_label_20, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui->screen_4_label_20, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui->screen_4_label_20, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui->screen_4_label_20, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui->screen_4_label_20, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_4_label_20, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write codes screen_4_label_19
    ui->screen_4_label_19 = lv_label_create(ui->screen_4_cont_1);
    lv_label_set_text(ui->screen_4_label_19, "Locating:2");
    lv_label_set_long_mode(ui->screen_4_label_19, LV_LABEL_LONG_WRAP);
    lv_obj_set_pos(ui->screen_4_label_19, 13, 140);
    lv_obj_set_size(ui->screen_4_label_19, 135, 27);

    //Write style for screen_4_label_19, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_border_width(ui->screen_4_label_19, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_4_label_19, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui->screen_4_label_19, lv_color_hex(0x000000), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->screen_4_label_19, &lv_font_montserrat_16, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->screen_4_label_19, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(ui->screen_4_label_19, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_line_space(ui->screen_4_label_19, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui->screen_4_label_19, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui->screen_4_label_19, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui->screen_4_label_19, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui->screen_4_label_19, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui->screen_4_label_19, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui->screen_4_label_19, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_4_label_19, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write codes screen_4_label_24
    ui->screen_4_label_24 = lv_label_create(ui->screen_4_cont_1);
    lv_label_set_text(ui->screen_4_label_24, "Number:");
    lv_label_set_long_mode(ui->screen_4_label_24, LV_LABEL_LONG_WRAP);
    lv_obj_set_pos(ui->screen_4_label_24, 18, 334);
    lv_obj_set_size(ui->screen_4_label_24, 113, 27);

    //Write style for screen_4_label_24, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_border_width(ui->screen_4_label_24, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_4_label_24, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui->screen_4_label_24, lv_color_hex(0x000000), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->screen_4_label_24, &lv_font_montserrat_16, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->screen_4_label_24, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(ui->screen_4_label_24, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_line_space(ui->screen_4_label_24, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui->screen_4_label_24, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui->screen_4_label_24, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui->screen_4_label_24, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui->screen_4_label_24, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui->screen_4_label_24, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui->screen_4_label_24, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_4_label_24, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write codes screen_4_label_23
    ui->screen_4_label_23 = lv_label_create(ui->screen_4_cont_1);
    lv_label_set_text(ui->screen_4_label_23, "Time:--");
    lv_label_set_long_mode(ui->screen_4_label_23, LV_LABEL_LONG_WRAP);
    lv_obj_set_pos(ui->screen_4_label_23, -60, 299);
    lv_obj_set_size(ui->screen_4_label_23, 320, 27);

    //Write style for screen_4_label_23, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_border_width(ui->screen_4_label_23, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_4_label_23, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui->screen_4_label_23, lv_color_hex(0x000000), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->screen_4_label_23, &lv_font_montserrat_16, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->screen_4_label_23, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(ui->screen_4_label_23, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_line_space(ui->screen_4_label_23, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui->screen_4_label_23, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui->screen_4_label_23, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui->screen_4_label_23, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui->screen_4_label_23, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui->screen_4_label_23, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui->screen_4_label_23, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_4_label_23, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write codes screen_4_label_22
    ui->screen_4_label_22 = lv_label_create(ui->screen_4_cont_1);
    lv_label_set_text(ui->screen_4_label_22, "Locating:3");
    lv_label_set_long_mode(ui->screen_4_label_22, LV_LABEL_LONG_WRAP);
    lv_obj_set_pos(ui->screen_4_label_22, 13, 268);
    lv_obj_set_size(ui->screen_4_label_22, 135, 27);

    //Write style for screen_4_label_22, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_border_width(ui->screen_4_label_22, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_4_label_22, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui->screen_4_label_22, lv_color_hex(0x000000), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->screen_4_label_22, &lv_font_montserrat_16, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->screen_4_label_22, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(ui->screen_4_label_22, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_line_space(ui->screen_4_label_22, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui->screen_4_label_22, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui->screen_4_label_22, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui->screen_4_label_22, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui->screen_4_label_22, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui->screen_4_label_22, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui->screen_4_label_22, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_4_label_22, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write codes screen_4_ddlist_4
    ui->screen_4_ddlist_4 = lv_dropdown_create(ui->screen_4_cont_1);
    lv_dropdown_set_options(ui->screen_4_ddlist_4, "1\n2\n3\n4\n5");
    lv_obj_set_pos(ui->screen_4_ddlist_4, 152, 85);
    lv_obj_set_size(ui->screen_4_ddlist_4, 130, 30);

    //Write style for screen_4_ddlist_4, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_text_color(ui->screen_4_ddlist_4, lv_color_hex(0x0D3055), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->screen_4_ddlist_4, &lv_font_montserrat_16, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->screen_4_ddlist_4, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui->screen_4_ddlist_4, 1, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui->screen_4_ddlist_4, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui->screen_4_ddlist_4, lv_color_hex(0xe1e6ee), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_side(ui->screen_4_ddlist_4, LV_BORDER_SIDE_FULL, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui->screen_4_ddlist_4, 8, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui->screen_4_ddlist_4, 6, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui->screen_4_ddlist_4, 6, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_4_ddlist_4, 3, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui->screen_4_ddlist_4, 28, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen_4_ddlist_4, lv_color_hex(0x000000), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_4_ddlist_4, LV_GRAD_DIR_NONE, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_4_ddlist_4, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write style state: LV_STATE_CHECKED for &style_screen_4_ddlist_4_extra_list_selected_checked
    static lv_style_t style_screen_4_ddlist_4_extra_list_selected_checked;
    ui_init_style(&style_screen_4_ddlist_4_extra_list_selected_checked);

    lv_style_set_border_width(&style_screen_4_ddlist_4_extra_list_selected_checked, 1);
    lv_style_set_border_opa(&style_screen_4_ddlist_4_extra_list_selected_checked, 255);
    lv_style_set_border_color(&style_screen_4_ddlist_4_extra_list_selected_checked, lv_color_hex(0xe1e6ee));
    lv_style_set_border_side(&style_screen_4_ddlist_4_extra_list_selected_checked, LV_BORDER_SIDE_FULL);
    lv_style_set_radius(&style_screen_4_ddlist_4_extra_list_selected_checked, 3);
    lv_style_set_bg_opa(&style_screen_4_ddlist_4_extra_list_selected_checked, 255);
    lv_style_set_bg_color(&style_screen_4_ddlist_4_extra_list_selected_checked, lv_color_hex(0x00a1b5));
    lv_style_set_bg_grad_dir(&style_screen_4_ddlist_4_extra_list_selected_checked, LV_GRAD_DIR_NONE);
    lv_obj_add_style(lv_dropdown_get_list(ui->screen_4_ddlist_4), &style_screen_4_ddlist_4_extra_list_selected_checked, LV_PART_SELECTED|LV_STATE_CHECKED);

    //Write style state: LV_STATE_DEFAULT for &style_screen_4_ddlist_4_extra_list_main_default
    static lv_style_t style_screen_4_ddlist_4_extra_list_main_default;
    ui_init_style(&style_screen_4_ddlist_4_extra_list_main_default);

    lv_style_set_max_height(&style_screen_4_ddlist_4_extra_list_main_default, 90);
    lv_style_set_text_color(&style_screen_4_ddlist_4_extra_list_main_default, lv_color_hex(0x0D3055));
    lv_style_set_text_font(&style_screen_4_ddlist_4_extra_list_main_default, &lv_font_montserrat_16);
    lv_style_set_text_opa(&style_screen_4_ddlist_4_extra_list_main_default, 255);
    lv_style_set_border_width(&style_screen_4_ddlist_4_extra_list_main_default, 1);
    lv_style_set_border_opa(&style_screen_4_ddlist_4_extra_list_main_default, 255);
    lv_style_set_border_color(&style_screen_4_ddlist_4_extra_list_main_default, lv_color_hex(0xe1e6ee));
    lv_style_set_border_side(&style_screen_4_ddlist_4_extra_list_main_default, LV_BORDER_SIDE_FULL);
    lv_style_set_radius(&style_screen_4_ddlist_4_extra_list_main_default, 3);
    lv_style_set_bg_opa(&style_screen_4_ddlist_4_extra_list_main_default, 255);
    lv_style_set_bg_color(&style_screen_4_ddlist_4_extra_list_main_default, lv_color_hex(0xffffff));
    lv_style_set_bg_grad_dir(&style_screen_4_ddlist_4_extra_list_main_default, LV_GRAD_DIR_NONE);
    lv_obj_add_style(lv_dropdown_get_list(ui->screen_4_ddlist_4), &style_screen_4_ddlist_4_extra_list_main_default, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write style state: LV_STATE_DEFAULT for &style_screen_4_ddlist_4_extra_list_scrollbar_default
    static lv_style_t style_screen_4_ddlist_4_extra_list_scrollbar_default;
    ui_init_style(&style_screen_4_ddlist_4_extra_list_scrollbar_default);

    lv_style_set_radius(&style_screen_4_ddlist_4_extra_list_scrollbar_default, 3);
    lv_style_set_bg_opa(&style_screen_4_ddlist_4_extra_list_scrollbar_default, 255);
    lv_style_set_bg_color(&style_screen_4_ddlist_4_extra_list_scrollbar_default, lv_color_hex(0x00ff00));
    lv_style_set_bg_grad_dir(&style_screen_4_ddlist_4_extra_list_scrollbar_default, LV_GRAD_DIR_NONE);
    lv_obj_add_style(lv_dropdown_get_list(ui->screen_4_ddlist_4), &style_screen_4_ddlist_4_extra_list_scrollbar_default, LV_PART_SCROLLBAR|LV_STATE_DEFAULT);

    //The custom code of screen_4.


    //Update current screen layout.
    lv_obj_update_layout(ui->screen_4);

    //Init events for screen.
    events_init_screen_4(ui);
}
