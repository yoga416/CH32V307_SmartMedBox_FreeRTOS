
#include "sht40.h"
#include "sht40_reg.h"
#include "system.h"
#include "debug.h"
#include "FreeRTOS.h"
#include "task.h"

#define  DEBUG_SHT40 
#define  SHT40_MEASUREMENT_WAIT_TIME  20    /* SHT40 测量约需 10ms，给定 20ms 绝对安全 */
#define  SHT40_INITED                 1
#define  SHT40_NOT_INITED              0

#define SHT40_I2C_ADDRESS            0x44 
#define SHT40_I2C_ADDRESS_WRITE     ((SHT40_I2C_ADDRESS << 1) | 0x00U) 
#define SHT40_I2C_ADDRESS_READ      ((SHT40_I2C_ADDRESS << 1) | 0x01U) 

#define IS_INITED      (SHT40_INITED == g_inited) 

/******************************variables***************************************/
static  int8_t      g_inited = SHT40_NOT_INITED; 
static  uint64_t    g_device_id = 0; 


static  uint32_t    cpu_sr;

static void internal_delay_ms(bsp_sht40_driver_t *const p_instance, uint32_t ms)
{
    /* 核心修复：空指针检查，防止 HardFault */
    if(p_instance && p_instance->p_os_timebase_interface && p_instance->p_os_timebase_interface->pf_os_delay_ms) {
        p_instance->p_os_timebase_interface->pf_os_delay_ms(ms);
    } else {
        Delay_Ms(ms); 
    }
}

static SHT40_Status_t sht40_read_data(bsp_sht40_driver_t *const p_sht40_instance, float *temperature, float *humidity)
{
    SHT40_Status_t result = SHT40_OK;
    uint8_t buf[6] = {0};

    // 基础校验：确保实例、接口及关键函数指针有效
    if (!IS_INITED || p_sht40_instance == NULL || p_sht40_instance->p_iic_interface == NULL) {
        return SHT40_ERROR;
    }
    
    iic_driver_interface_t *i2c = p_sht40_instance->p_iic_interface;

    /* --- 第一阶段：发送测量指令 --- */
    i2c->pf_iic_start(NULL);
    if (i2c->pf_iic_send_byte(NULL, SHT40_I2C_ADDRESS_WRITE) != SHT40_OK ||
        i2c->pf_iic_wait_ack(NULL) != SHT40_OK) {
        i2c->pf_iic_stop(NULL);
        return SHT40_ERROR;
    }
    
    // 关键修复：必须使用写地址发送测量命令，否则传感器不会响应
    i2c->pf_iic_send_byte(NULL, SHT40_MEASURE_CMD);
    i2c->pf_iic_wait_ack(NULL);
    i2c->pf_iic_stop(NULL);

    /* --- 第二阶段：测量延时 --- */
    // 等待传感器完成物理转换
    internal_delay_ms(p_sht40_instance, SHT40_MEASUREMENT_WAIT_TIME); 

    /* --- 第三阶段：读取结果 (6个字节) --- */
    i2c->pf_iic_start(NULL);
    if (i2c->pf_iic_send_byte(NULL, SHT40_I2C_ADDRESS_READ) != SHT40_OK ||
        i2c->pf_iic_wait_ack(NULL) != SHT40_OK) {
        i2c->pf_iic_stop(NULL);
        return SHT40_ERROR;
    }
    
    // 循环读取数据包 [Temp_MSB, Temp_LSB, Temp_CRC, Hum_MSB, Hum_LSB, Hum_CRC]
    for (int i = 0; i < 6; i++) {
        i2c->pf_iic_read_byte(NULL, &buf[i]);
        
        // 关键逻辑：除了最后一个字节发送 NACK，其余都必须发送 ACK
        if (i < 5) {
            if (i2c->pf_iic_send_ack) {
                i2c->pf_iic_send_ack(NULL);
            }
        } else {
            if (i2c->pf_iic_send_no_ack) {
                i2c->pf_iic_send_no_ack(NULL);
            }
        }
    }   
    i2c->pf_iic_stop(NULL);

    /* --- 第四阶段：物理量转换 --- */
    if (temperature) {
        uint16_t t_ticks = ((uint16_t)buf[0] << 8) | buf[1];
        if (t_ticks == 0xFFFF || t_ticks == 0x0000) {
            *temperature = -999.0f; // 标记错误值
        } else {
            // 公式：T = -45 + 175 * (S_T / (2^16 - 1))
            *temperature = -45.0f + 175.0f * (float)t_ticks / 65535.0f;
        }
    }

    if (humidity) {
        uint16_t h_ticks = ((uint16_t)buf[3] << 8) | buf[4];
        if (h_ticks == 0xFFFF || h_ticks == 0x0000) {
            *humidity = -999.0f;
        } else {
            // 公式：RH = -6 + 125 * (S_RH / (2^16 - 1))
            float rh_val = -6.0f + 125.0f * (float)h_ticks / 65535.0f;
            // 限制在 0-100% 范围内
            if (rh_val > 100.0f) rh_val = 100.0f;
            if (rh_val < 0.0f) rh_val = 0.0f;
            *humidity = rh_val;
        }
    }

    return SHT40_OK;
}

static SHT40_Status_t sht40_init(bsp_sht40_driver_t *const p_inst)
{
    SHT40_Status_t result = SHT40_OK;

    // 1. 基础指针检查
    if (p_inst == NULL || p_inst->p_iic_interface == NULL) {
        return SHT40_ERROR;
    }

    // 2. 检查是否已经初始化，避免重复开启时钟
    if (IS_INITED) return SHT40_OK; 

    // 3. 【关键：执行底层 I2C 初始化】
    // 这行代码会调用 SHT40_IIC_Init，从而开启 RCC 时钟并配置 PC0/PA1 为开漏输出
    if (p_inst->p_iic_interface->pf_iic_init != NULL) {
        result = p_inst->p_iic_interface->pf_iic_init(NULL);
        if (result != SHT40_OK) {
            return SHT40_ERROR;
        }
    }

    // 5. 标记初始化成功
    g_inited = SHT40_INITED; 

    return SHT40_OK;
}

SHT40_Status_t SHT40_inst(bsp_sht40_driver_t *const p_inst, iic_driver_interface_t *const p_iic, 
                        os_timebase_interface_t *const p_os, timebase_interface_t *const p_tb)
{
    if (p_inst == NULL) return SHT40_ERROR; 
    p_inst->p_iic_interface = p_iic;
    p_inst->p_os_timebase_interface = p_os;
    p_inst->p_timebase_interface = p_tb;
    p_inst->pf_read = sht40_read_data;
    return sht40_init(p_inst);
}


SHT40_Status_t SHT40_Read(SHT40_Data_t *data)
{
    static bsp_sht40_driver_t s_sht40_instance;
    static uint8_t s_inst_inited = 0U;
    
    if (data == NULL) return SHT40_ERROR;

    if (s_inst_inited == 0U) {
        // 外部全局变量引用（假设你在 port.c 定义过）
        extern os_timebase_interface_t g_sht40_freertos_if; 
        extern timebase_interface_t g_sht40_timebase_interface;

        SHT40_inst(&s_sht40_instance, NULL, &g_sht40_freertos_if, &g_sht40_timebase_interface);
        s_inst_inited = 1U;
    }

    return s_sht40_instance.pf_read(&s_sht40_instance, &data->temperature_c, &data->humidity_rh);
}