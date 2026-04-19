#include "bsp_battery_handler.h"
#include <stdio.h>
#include <string.h>

bsp_bat_handler_t g_bat_handler_instance;

/* 101点极致放电曲线表 (索引 i 直接对应电量: 100 - i) */
static const uint16_t Battery_Voltage_Table[101] = {
    4200, 4182, 4164, 4146, 4128, 4110, 4100, 4090, 4080, 4070, // 100% - 91%
    4060, 4052, 4044, 4036, 4028, 4020, 4012, 4004, 3996, 3988, // 90% - 81%
    3980, 3974, 3968, 3962, 3956, 3950, 3944, 3938, 3932, 3926, // 80% - 71%
    3920, 3915, 3910, 3905, 3900, 3895, 3890, 3885, 3880, 3875, // 70% - 61%
    3870, 3865, 3860, 3855, 3850, 3845, 3840, 3835, 3830, 3825, // 60% - 51%
    3820, 3817, 3814, 3811, 3808, 3805, 3802, 3799, 3796, 3793, // 50% - 41%
    3790, 3788, 3786, 3784, 3782, 3780, 3778, 3776, 3774, 3772, // 40% - 31%
    3770, 3767, 3764, 3761, 3758, 3755, 3752, 3749, 3746, 3743, // 30% - 21%
    3740, 3734, 3728, 3722, 3716, 3710, 3704, 3698, 3692, 3686, // 20% - 11%
    3680, 3672, 3664, 3656, 3648, 3640, 3630, 3610, 3580, 3520, // 10% - 1%
    3400                                                        // 0%
};

/* 高精度查表算法 */
static uint8_t calculate_battery_soc_ultra_precision(uint16_t voltage_mv) {
    if (voltage_mv >= Battery_Voltage_Table[0])   return 100;
    if (voltage_mv <= Battery_Voltage_Table[100]) return 0;

    for (int i = 0; i < 100; i++) {
        if (voltage_mv <= Battery_Voltage_Table[i] && voltage_mv > Battery_Voltage_Table[i+1]) {
            uint16_t v_diff = Battery_Voltage_Table[i] - Battery_Voltage_Table[i+1];
            uint8_t base_soc = 99 - i;
            uint16_t offset = voltage_mv - Battery_Voltage_Table[i+1];
            
            // 四舍五入补偿
            if (offset >= (v_diff / 2)) {
                return base_soc + 1;
            } else {
                return base_soc;
            }
        }
    }
    return 0; 
}

void bsp_battery_handler_inst(bsp_bat_handler_t *handler, bsp_battery_driver_t *driver, bat_handler_os_instance_t *os) {
    handler->bat_driver_instance = driver;
    handler->os_handler_instance = os;
}

void battery_handler_task(void *argument) {
    bsp_bat_handler_t *handler = (bsp_bat_handler_t *)argument;
    uint16_t voltage_mv = 0;
    uint8_t  current_calc_soc = 0;
    static uint8_t last_displayed_soc = 255; 
    // static Packet_t packet_bat; // 如果开启了通信协议，解除注释

    for (;;) {
        // 1. 获取极致滤波后的真实电压
        if (handler->bat_driver_instance->pf_get_voltage_mv(handler->bat_driver_instance, &voltage_mv) == BAT_DRIVER_OK) {
            
            // 2. 查表计算物理电量
            current_calc_soc = calculate_battery_soc_ultra_precision(voltage_mv);
            
            // 3. 单调递减锁 (不充电时，电量只降不升)
            if (last_displayed_soc == 255) {
                last_displayed_soc = current_calc_soc;
            } else {
                if (current_calc_soc > last_displayed_soc) {
                    current_calc_soc = last_displayed_soc; // 屏蔽反弹假象
                } else {
                    last_displayed_soc = current_calc_soc;
                }
            }

            printf("\r\n[Battery] Stable Vol: %dmV, SOC: %d%%\r\n", voltage_mv, current_calc_soc);

            // 4. 封包上传逻辑 (根据你的实际宏定义调整)
            /*
            memset(&packet_bat, 0x00, sizeof(Packet_t));
            packet_bat.head[0] = PACKET_HEAD;
            packet_bat.sensor_num = 1;
            packet_bat.sensor_data[0].sensor_id = 0x05; // 假定电量 ID
            packet_bat.sensor_data[0].data = (uint16_t)current_calc_soc;
            packet_bat.length = sizeof(Packet_t);       
            packet_bat.crc = Calculate_CRC(&packet_bat);
            packet_bat.tail[0] = PACKET_TAIL;
            
            RingBuffer_push(&g_ring_buffer, &packet_bat);
            */
        }

        // 5. 极低频检测，每 10 秒唤醒一次 (10000ms)，极其省电
        if (handler->os_handler_instance && handler->os_handler_instance->pf_os_delay_ms) {
            handler->os_handler_instance->pf_os_delay_ms(60000); // 60秒检测一次，实际使用中可以根据需求调整频率
        }
    }
}