/**
 * @file bsp_MLX_90614.c
 * @brief MLX90614 Software I2C driver implementation.
 */

#include "bsp_MLX_90614.h"
#include "debug.h"
#include "bsp_MLX_90614_reg.h"
#include "FreeRTOS.h"
#include "task.h"

#define  MLX_INITED                          1
#define  MLX_NOT_INITED                      0
#define  IS_INITED      (MLX_INITED == g_inited)

static int8_t g_inited = MLX_NOT_INITED;

/* ======== 使用软件 IIC 读取寄存器原始数据 ======== */
static MLX90614_Status_t MLX90614_ReadRawTemp(bsp_mlx90614_driver_t *driver, uint8_t reg_addr, uint16_t *raw_temp)
{
    uint8_t data_l = 0, data_h = 0, pec = 0;
    mlx_iic_driver_instance_t *iic = driver->p_iic_instance;

    if (iic == NULL) return MLX90614_ERROR;
    
    // 进入临界区，禁止任务切换，保证 I2C 通信的原子性
    vTaskSuspendAll();
    // 1. 发送起始信号
    iic->pf_iic_start(driver);

    // 2. 发送从机地址 (写)
    iic->pf_iic_send_byte(driver, MLX90614_ADDR_WRITE);
    if (iic->pf_iic_wait_ack(driver) != MLX90614_OK) goto error;

    // 3. 发送寄存器地址
    iic->pf_iic_send_byte(driver, reg_addr);
    if (iic->pf_iic_wait_ack(driver) != MLX90614_OK) goto error;

    // 4. 发送重复起始信号
    iic->pf_iic_start(driver);

    // 5. 发送从机地址 (读)
    iic->pf_iic_send_byte(driver, MLX90614_ADDR_READ);
    if (iic->pf_iic_wait_ack(driver) != MLX90614_OK) goto error;

    // 6. 读低 8 位并发送 ACK
    iic->pf_iic_rec_byte(driver, &data_l);
    iic->pf_iic_send_ack(driver);

    // 7. 读高 8 位并发送 ACK
    iic->pf_iic_rec_byte(driver, &data_h);
    iic->pf_iic_send_ack(driver);

    // 8. 读 PEC 校验码并发送 NACK
    iic->pf_iic_rec_byte(driver, &pec);
    iic->pfiic_send_no_ack(driver);

    // 9. 发送停止信号
    iic->pf_iic_stop(driver);

    // 恢复任务调度
    xTaskResumeAll();
    *raw_temp = (uint16_t)(((uint16_t)data_h << 8) | data_l);
    return MLX90614_OK;

error:
    iic->pf_iic_stop(driver);
    xTaskResumeAll();
    return MLX90614_TIMEOUT;
}

static MLX90614_Status_t MLX90614_ReadAmbientTempC(bsp_mlx90614_driver_t *driver, float *temp)
{
    uint16_t raw;
    MLX90614_Status_t status = MLX90614_ReadRawTemp(driver, MLX90614_REG_TA, &raw);
    if (status == MLX90614_OK) {
        *temp = ((float)raw * 0.02f) - 273.15f;
    }
    return status;
}

static MLX90614_Status_t MLX90614_ReadObjectTempC(bsp_mlx90614_driver_t *driver, float *temp)
{
    uint16_t raw;
    MLX90614_Status_t status = MLX90614_ReadRawTemp(driver, MLX90614_REG_TOBJ1, &raw);
    if (status == MLX90614_OK) {
        *temp = ((float)raw * 0.02f) - 273.15f;
    }
    return status;
}

/* ================= 接口实现 ================= */
static MLX90614_Status_t MLX90614_driver_init(void *context)
{
    bsp_mlx90614_driver_t *driver = (bsp_mlx90614_driver_t *)context;
    if (driver == NULL) return MLX90614_ERROR;
    
    if(IS_INITED) return MLX90614_OK;

    if (driver->p_iic_instance && driver->p_iic_instance->pf_iic_init) {
        driver->p_iic_instance->pf_iic_init(driver);
    }

    g_inited = MLX_INITED;
    return MLX90614_OK;
}

static MLX90614_Status_t MLX90614_driver_deinit(void *context)
{
    g_inited = MLX_NOT_INITED;
    return MLX90614_OK;   
}

static MLX90614_Status_t MLX90614_ReadID(void *context, uint8_t * const id_buf, uint8_t buf_len) {
    return MLX90614_OK;
}   

static MLX90614_Status_t read_data_raw(void *context, float * const temp) {
    return MLX90614_ReadObjectTempC((bsp_mlx90614_driver_t *)context, temp);
}

static MLX90614_Status_t read_surface_temp(void *context, float * const temp) {
    return MLX90614_ReadAmbientTempC((bsp_mlx90614_driver_t *)context, temp);
}

static MLX90614_Status_t read_body_temp(void *context, float * const temp) {
    return MLX90614_ReadObjectTempC((bsp_mlx90614_driver_t *)context, temp);
}

static MLX90614_Status_t sleep(void *context) { return MLX90614_OK; }
static MLX90614_Status_t wake(void *context) { return MLX90614_OK; }

MLX90614_Status_t bsp_mlx_90614_inst(
            bsp_mlx90614_driver_t * const pf_driver,
            mlx_iic_driver_instance_t  *p_iic_instance,
#ifdef OS_SUPPORTING
            mlx_os_timebase_interface_t  *p_os_timebase_instance,
#endif
            mlx_timebase_interface_t   *p_timebase_instance)
{
    if (pf_driver == NULL || p_timebase_instance == NULL || p_iic_instance == NULL) return MLX90614_ERROR;

    pf_driver->p_iic_instance = p_iic_instance;
    pf_driver->p_timebase_instance = p_timebase_instance;
#ifdef OS_SUPPORTING
    pf_driver->p_os_timebase_instance = p_os_timebase_instance;
#endif
    pf_driver->pf_init = MLX90614_driver_init;
    pf_driver->pf_deinit = MLX90614_driver_deinit;
    pf_driver->pf_read_id = MLX90614_ReadID;
    pf_driver->pf_read_data_raw = read_data_raw;
    pf_driver->pf_read_surface_temp = read_surface_temp;
    pf_driver->pf_read_body_temp = read_body_temp;
    pf_driver->pf_sleep = sleep;
    pf_driver->pf_wake = wake;

    return pf_driver->pf_init(pf_driver);
}