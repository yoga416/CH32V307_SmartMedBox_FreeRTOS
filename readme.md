- $ # 强制让本地分支重置到远程分支的状态
	git reset --hard origin/main

- 

- 

- [smartmedBox_v1.11_5.1_update](..\smartmedBox_v1.11_5.1_update) 

- 这一版是为了加上lcd和lvgl的所改进的一版。

	在lvgl中药切换界面的时候就要清除引脚

	在event_init.c文件改了，就避免界面卡死了

	```C
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
	#include "lvgl.h"
	
	#if LV_USE_GUIDER_SIMULATOR && LV_USE_FREEMASTER
	#include "freemaster_client.h"
	#endif
	
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
	```

	

![image-20260522204334831](C:\Users\yoga1\AppData\Roaming\Typora\typora-user-images\image-20260522204334831.png)在

在这个文件中改大小



![image-20260522205608015](C:\Users\yoga1\AppData\Roaming\Typora\typora-user-images\image-20260522205608015.png)

这里是字符库，可以更改字符

---

# 🏥 CH32V307 智能药盒 — 已完成功能清单

## 一、硬件外设

| 外设 | 接口 | 用途 |
|------|------|------|
| **MAX30102** | I2C | 心率 (HR) & 血氧 (SpO₂) 测量 |
| **MLX90614** | I2C | 非接触式红外体温测量 |
| **SHT40** | I2C | 环境温湿度测量 |
| **A7670C (4G)** | USART1 | 短信报警发送（"请按时服药！"） |
| **ESP8266 (WiFi)** | USART2+DMA | 云端数据同步、RTC时间同步、天气同步 |
| **天问ASR模块** | USART3(PD8/PD9) | 语音识别指令接收 + 语音播报合成 |
| **FT6336 触摸屏** | 软件I2C | 触摸交互输入 |
| **TFT LCD (320×480)** | FSMC/SPI | LVGL GUI 彩色显示 |
| **W25Q64 SPI Flash** | SPI2 | 数据持久化存储 |
| **片内RTC** | LSI时钟 | 实时时钟、闹钟定时 |
| **独立看门狗 (IWDG)** | 片内外设 | 系统防死机保护 |

---

## 二、FreeRTOS 任务（共11个）

| 任务名 | 优先级 | 栈大小 | 功能 |
|--------|--------|--------|------|
| **`app_task`** | 3 | 2KB | 主应用逻辑：处理事件队列、定时喂狗 |
| **`bsp_sensor_task`** | 2 | 4KB | 传感器指令执行：接收 `xSysCmdQueue` 指令，触发传感器测量和短信报警 |
| **`usart_task`** | 4 | 1KB | WiFi通信：收发ESP8266数据，处理时间/位置/天气/心跳指令 |
| **`tianwen_task`** | 4 | 2KB | 语音模块通信：解析5字节协议帧，识别6种语音指令，播报传感器数据 |
| **`lcd_task`** | 3 | 4KB | LVGL定时器驱动：每5ms调用 `lv_tick_inc()` + `lv_timer_handler()` |
| **`sensor_lcd_task`** | 1 | 4KB | 数据显示+存储：接收传感器队列 → 追加历史 → 存Flash → 刷新UI |
| **`watchdog_task`** | 4 | 512B | 喂狗守护：等待心跳通知，2秒超时则系统复位 |
| **`Battery_Task`** | — | 256 words | 电池检测：60秒采ADC，中位值平均滤波，放电曲线查表 |
| **MAX30102 Handler** | — | 2KB | 心率血氧算法数据处理 |
| **SHT40 Handler** | — | 2KB | 环境温湿度数据处理 |
| **MLX90614 Handler** | — | 2KB | 红外体温数据处理 |

---

## 三、LVGL 用户界面（5个屏幕）

### 📺 Screen 0 — 主页面
- 显示 WiFi 连接状态（T/F）
- 实时时钟 HH:MM:SS
- 当前日期 YYYY/MM/DD
- 环境温度 & 环境湿度
- "Next" 按钮 → 进入功能菜单

### 📺 Screen 1 — 功能菜单
- **"查看记录"** → Screen 2（健康数据 & 漏服记录）
- **"吃药"** → Screen 3（吃药确认）
- **"用药方案"** → Screen 4（设置）
- **返回** → 主页面

### 📺 Screen 2 — 健康数据 & 漏服记录
- 显示4条漏服记录（最新→最旧）
- 显示最新心率、血氧、体温数值
- 返回按钮

### 📺 Screen 3 — 吃药确认
- 到吃药时间：显示3个药品按钮，点击标记"已服"
- 未到吃药时间：显示"不在用药时间哦"
- 返回按钮

### 📺 Screen 4 — 用药方案设置
- 显示3个吃药时间（Time: HH:MM）
- 3个下拉选择器，设置每种药的颗数
- 保存按钮 → 存Flash + 更新RTC闹钟

---

## 四、数据记录与历史追溯

| 数据类型 | 最大保存条数 | 说明 |
|----------|-------------|------|
| ❤️ 心率历史 | 10条 | 带时间戳，满10条覆盖最旧 |
| 🩸 血氧历史 | 10条 | 与心率共用结构体 |
| 🌡️ 体温历史 | 10条 | 带时间戳 |
| 🌱 环境温湿度历史 | 10条 | 带时间戳 |
| ⚠️ 漏服记录 | 4条 | 最新在最前，环形覆盖 |

- 每次追加新数据 → 自动保存到 W25Q Flash（地址 `0x7F8000`）
- 开机自动从 Flash 恢复所有历史数据和配置

---

## 五、云端 & 通信

### 🌐 WiFi (ESP8266)
- 云端时间同步（每200秒校准RTC）
- 天气同步（天气代码 + 温度）
- 位置同步（城市名称）
- WiFi心跳检测（20秒超时断线标记）
- 界面显示连接状态

### 📱 4G (A7670C) 短信报警
- 发送 "请按时服药！" 短信至预设监护人号码
- 支持模块休眠/唤醒电源管理

### 🎤 天问语音交互
- **语音识别**：人脸识别、健康检测、体温检测、心率血氧、添加人脸
- **语音播报**：吃药提醒、体温/心率/血氧数值播报、欢迎语、闹钟提示

---

## 六、闹钟与吃药提醒

- 3组可配置吃药时间（时/分/颗数）
- RTC硬件闹钟：自动计算最近的下一次吃药时间
- 吃药状态跟踪（`med_status[3]`：0=未服, 1=已服）
- 每日凌晨00:00自动重置服药状态
- 每10秒检查漏服，自动记录漏服条目
- 语音播报提醒 + 短信报警双重保障

---

## 七、系统保护机制

| 机制 | 说明 |
|------|------|
| **独立看门狗 (IWDG)** | 2秒超时系统复位，防止死机 |
| **LVGL互斥锁** | `xGuiMutex` 保护多任务并发访问LVGL |
| **野指针清理** | 页面切换时 `clear_all_old_pointers()` 全局对象置NULL |
| **`lv_obj_is_valid()`** | 所有UI更新前检查对象有效性 |
| **ADC滤波** | 电池检测：50次采样 + 中位值平均滤波 |
| **电量防反弹** | 单调递减滤波器，电量只降不升 |

---

## 八、系统架构

```
语音指令 ──→ tianwen_task ──→ xSysCmdQueue ──→ bsp_sensor_task ──→ 传感器Handler
                                                      │
                                                      ↓
                                               sensor_data_lcd_queue
                                                      │
WiFi指令 ──→ usart_task ──────────────────→ sensor_lcd_task ──→ LVGL UI 刷新
用户触屏 ──→ lcd_task(LVGL)                                    │
                                                                 ↓
                                                              W25Q Flash
```

系统通过4个队列（`xSysCmdQueue`、`sensor_data_lcd_queue`、`sendtianwenQueue`、`CmdQueue`）+ 事件队列 `xAppEventQueue` 解耦所有模块。

