#include "bsp_max_30102_driver.h"
#include <string.h>

static void internal_delay(bsp_max30102_driver_t *p, uint32_t ms) {
    if (p->sys && p->sys->pf_delay_ms) 
    {
        p->sys->pf_delay_ms(ms);
    }else
    {
        volatile uint32_t count = ms * 1000; 
        while (count--) {
            __asm__("nop");
        }
    }
}

static MAX_DRIVER_Status_t max30102_reset(bsp_max30102_driver_t *p) {
    // 复位寄存器，等待 100ms，确保设备重启完成
    p->hw->pf_write_reg(p->hw->hi2c, MAX30102_I2C_ADDR, REG_MODE_CONFIG, 0x40);
    internal_delay(p, 100);
    return MAX_DRIVER_OK;
}

static MAX_DRIVER_Status_t max30102_init(bsp_max30102_driver_t *p) {
    uint8_t id = 0;
    void *ctx = p->hw->hi2c;

    p->pf_reset(p);

    // 1. 验证 ID
    p->hw->pf_read_reg(ctx, MAX30102_I2C_ADDR, REG_PART_ID, &id);
    if (id != 0x15)
    {
#ifdef DEBUG_ENABLE
        printf("[MAX30102] Device ID mismatch: expected 0x15, got 0x%02X\n", id);
#endif    
    return MAX_DRIVER_ERROR;
    }           

    // 2. 中断配置：开启新数据就绪中断 (依然需要，用来触发 PC6)
    p->hw->pf_write_reg(ctx, MAX30102_I2C_ADDR, REG_INTR_ENABLE_1, 0xC0); 
    p->hw->pf_write_reg(ctx, MAX30102_I2C_ADDR, REG_INTR_ENABLE_2, 0x00);

    // 3. FIFO 配置
    p->hw->pf_write_reg(ctx, MAX30102_I2C_ADDR, REG_FIFO_WR_PTR, 0x00);
    p->hw->pf_write_reg(ctx, MAX30102_I2C_ADDR, REG_OVF_COUNTER, 0x00);
    p->hw->pf_write_reg(ctx, MAX30102_I2C_ADDR, REG_FIFO_RD_PTR, 0x00);
    p->hw->pf_write_reg(ctx, MAX30102_I2C_ADDR, REG_FIFO_CONFIG, 0x4F);

    // 4. 工作模式与参数
    p->hw->pf_write_reg(ctx, MAX30102_I2C_ADDR, REG_MODE_CONFIG, 0x03);
    p->hw->pf_write_reg(ctx, MAX30102_I2C_ADDR, REG_SPO2_CONFIG, 0x27); // 50Hz
    p->hw->pf_write_reg(ctx, MAX30102_I2C_ADDR, REG_LED1_PA, 0x3F);
    p->hw->pf_write_reg(ctx, MAX30102_I2C_ADDR, REG_LED2_PA, 0x3F);

    // 清除一次初始中断状态
    uint8_t dummy;
    p->hw->pf_read_reg(ctx, MAX30102_I2C_ADDR, REG_INTR_STATUS_1, &dummy);

    return MAX_DRIVER_OK;
}

/* 阻塞读 6 字节数据，并立即返回平滑滤波后的结果 */
static MAX_DRIVER_Status_t max30102_get_filtered(bsp_max30102_driver_t *p, uint32_t *red_f, uint32_t *ir_f) {
    uint8_t buf[6] = {0};

    if (!p->hw->pf_read_mem) 
    {
#ifdef DEBUG_ENABLE
        printf("[MAX30102] Error: pf_read_mem function pointer is NULL. Please implement this function for blocking read.\n");
#endif // DEBUG_ENABLE
        return MAX_DRIVER_ERROR;
    }

    // 1. 阻塞读取 6 字节 FIFO (读取 FIFO 后硬件会自动清空中断状态，PC6 恢复高电平)
    if (p->hw->pf_read_mem(p->hw->hi2c, MAX30102_I2C_ADDR, REG_FIFO_DATA, buf, 6) != MAX_DRIVER_OK) {

#ifdef DEBUG_ENABLE
        printf("[MAX30102] Error: Failed to read FIFO data.\n");
#endif // DEBUG_ENABLE

        return MAX_DRIVER_ERROR;
    }

    // 2. 拼接有效数据
    uint32_t r_raw = (((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2]) & 0x03FFFF;
    uint32_t i_raw = (((uint32_t)buf[3] << 16) | ((uint32_t)buf[4] << 8) | buf[5]) & 0x03FFFF;

    max_filter_data_t *f = &p->filter_state;

    // 3. 滑动窗口维护
    if (f->window_count < MAX30102_FILTER_WINDOW) {
        f->window_count++;
    }
     else {
        f->red_sum -= f->red_window[f->window_index];
        f->ir_sum  -= f->ir_window[f->window_index];
    }
    
    f->red_window[f->window_index] = r_raw;
    f->ir_window[f->window_index]  = i_raw;
    f->red_sum += r_raw;
    f->ir_sum  += i_raw;
    
    *red_f = f->red_sum / f->window_count;
    *ir_f  = f->ir_sum  / f->window_count;
    
    // 4. 更新窗口索引
    if (++f->window_index >= MAX30102_FILTER_WINDOW) 
    {
        f->window_index = 0;
    }
    
    return MAX_DRIVER_OK;
}

MAX_DRIVER_Status_t bsp_max30102_inst(bsp_max30102_driver_t *p_driver, max_hw_interface_t *hw, max_sys_interface_t *sys) {
    memset(p_driver, 0, sizeof(bsp_max30102_driver_t));
    p_driver->hw  = hw;
    p_driver->sys = sys;

    p_driver->pf_init = max30102_init;
    p_driver->pf_reset = max30102_reset;
    p_driver->pf_get_filtered = max30102_get_filtered;

    return p_driver->pf_init(p_driver);
}