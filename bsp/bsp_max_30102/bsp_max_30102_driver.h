#ifndef _BSP_MAX30102_DRIVER_H_
#define _BSP_MAX30102_DRIVER_H_

#include <stdint.h>
#include <stddef.h>

#define DEBUG_ENABLE 
/* ========================================================================= */
/* ======================== 寄存器地址与宏定义 ============================= */
/* ========================================================================= */
#define MAX30102_I2C_ADDR        (0x57 << 1)

#define REG_INTR_STATUS_1        0x00
#define REG_INTR_STATUS_2        0x01
#define REG_INTR_ENABLE_1        0x02
#define REG_INTR_ENABLE_2        0x03
#define REG_FIFO_WR_PTR          0x04
#define REG_OVF_COUNTER          0x05
#define REG_FIFO_RD_PTR          0x06
#define REG_FIFO_DATA            0x07
#define REG_FIFO_CONFIG          0x08
#define REG_MODE_CONFIG          0x09
#define REG_SPO2_CONFIG          0x0A
#define REG_LED1_PA              0x0C 
#define REG_LED2_PA              0x0D 
#define REG_PART_ID              0xFF 

#define MAX30102_FILTER_WINDOW   20

typedef enum {
    MAX_DRIVER_OK      = 0,
    MAX_DRIVER_ERROR   = 1,
    MAX_DRIVER_TIMEOUT = 2,
    MAX_DRIVER_PARAM   = 3,
} MAX_DRIVER_Status_t;

/* 滑动滤波状态缓存 */
typedef struct {
    uint32_t red_window[MAX30102_FILTER_WINDOW];
    uint32_t ir_window[MAX30102_FILTER_WINDOW];
    uint8_t  window_index;
    uint8_t  window_count;
    uint32_t red_sum;
    uint32_t ir_sum;
} max_filter_data_t;

/* ========================================================================= */
/* ======================== 硬件接口抽象 =================================== */
/* ========================================================================= */
typedef struct {
    void *hi2c; 
    MAX_DRIVER_Status_t (*pf_write_reg)(void *context, uint16_t dev_addr, uint8_t reg, uint8_t data);
    MAX_DRIVER_Status_t (*pf_read_reg)(void *context, uint16_t dev_addr, uint8_t reg, uint8_t *p_data);
    MAX_DRIVER_Status_t (*pf_read_mem)(void *context, uint16_t dev_addr, uint8_t reg, uint8_t *p_data, uint16_t size);
} max_hw_interface_t;

typedef struct {
    void (*pf_delay_ms)(uint32_t ms);
} max_sys_interface_t;

typedef struct bsp_max30102_driver_t {
    max_hw_interface_t   *hw;
    max_sys_interface_t  *sys;
    max_filter_data_t    filter_state;

    MAX_DRIVER_Status_t (*pf_init)(struct bsp_max30102_driver_t *p_driver);
    MAX_DRIVER_Status_t (*pf_reset)(struct bsp_max30102_driver_t *p_driver);
    MAX_DRIVER_Status_t (*pf_get_filtered)(struct bsp_max30102_driver_t *p_driver, uint32_t *red_f, uint32_t *ir_f);
} bsp_max30102_driver_t;


MAX_DRIVER_Status_t bsp_max30102_inst(bsp_max30102_driver_t *p_driver,
                                      max_hw_interface_t *hw, 
                                      max_sys_interface_t *sys);

#endif /* _BSP_MAX30102_DRIVER_H_ */