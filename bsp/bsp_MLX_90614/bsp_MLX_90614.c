/**
 * @file bsp_MLX_90614.c
 * @brief MLX90614 hardware I2C2 driver implementation.
 * * This driver provides high-level functions to interface with the MLX90614
 * infrared thermometer using the CH32V30x hardware I2C peripheral.
 */

#include "bsp_MLX_90614.h"
#include "debug.h"
#include "ch32v30x_i2c.h"
#include "bsp_MLX_90614_reg.h"
#include "FreeRTOS.h"
#include "task.h"

#define  MLX_INITED                          1
#define  MLX_NOT_INITED                      0
#define IS_INITED      (MLX_INITED == g_inited)   /**< 检查 SHT40 是否已初始化的宏 */

/******************************variables***************************************/
static  int8_t      g_inited = MLX_NOT_INITED; // 追踪 SHT40 初始化状态的标志
/**
 * @brief Wait for a specific I2C event to complete.
 * @note This is a blocking call that consumes CPU resources. Ensure bus health 
 * to prevent infinite loops.
 * @param event The expected I2C event.
 * @return MLX90614_Status_t Status of the wait operation.
 */
static MLX90614_Status_t MLX_WaitEvent(uint32_t event)
{
    uint32_t timeout = MLX_I2C_TIMEOUT;
    
    while ((I2C_GetLastEvent(MLX_I2C) & event) != event) {
        if (timeout-- == 0U) {
            return MLX90614_TIMEOUT;
        }
    }
    return MLX90614_OK;
}

/**
 * @brief Wait for a specific I2C flag to be set.
 * @param flag The I2C flag to check (e.g., I2C_FLAG_RXNE).
 * @return MLX90614_Status_t Status of the wait operation.
 */
static MLX90614_Status_t MLX_WaitFlagSet(uint32_t flag)
{
    uint32_t timeout = MLX_I2C_TIMEOUT;
    while (I2C_GetFlagStatus(MLX_I2C, flag) == RESET) {
        if (timeout-- == 0U) {
            return MLX90614_TIMEOUT;
        }
    }
    return MLX90614_OK;
}

/**
 * @brief Wait for the I2C bus to become free.
 * @return MLX90614_Status_t MLX90614_OK if free, MLX90614_TIMEOUT otherwise.
 */
static MLX90614_Status_t MLX_WaitBusFree(void)
{
    uint32_t timeout = MLX_I2C_TIMEOUT;

    while (I2C_GetFlagStatus(MLX_I2C, I2C_FLAG_BUSY) == SET) {
        if (timeout-- == 0U) {
            return MLX90614_TIMEOUT;
        }
    }
    return MLX90614_OK;
}

/**
 * @brief Initializes the MLX90614 communication interface (Hardware I2C).
 * @return MLX90614_Status_t Initialization status.
 */
MLX90614_Status_t MLX90614_SoftI2C_Init(void)
{
    uint32_t timeout = 0;
    GPIO_InitTypeDef gpio_init = {0};
    I2C_InitTypeDef i2c_init = {0};
    
    /* Enable GPIO and I2C Peripheral Clocks */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);

    /* Configure I2C2 Pins: SCL (PB10) and SDA (PB11) */
    gpio_init.GPIO_Pin = MLX_SCL_PIN | MLX_SDA_PIN;
    gpio_init.GPIO_Mode = GPIO_Mode_AF_OD; // Alternate Function Open-Drain
    gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio_init);

    /* I2C Configuration Parameters */
    i2c_init.I2C_ClockSpeed = 100000;                      // 100kHz Standard Mode
    i2c_init.I2C_Mode = I2C_Mode_I2C;
    i2c_init.I2C_DutyCycle = I2C_DutyCycle_2;
    i2c_init.I2C_OwnAddress1 = 0x00;                       // Host address (unused in Master mode)
    i2c_init.I2C_Ack = I2C_Ack_Enable;                     // Enable ACK
    i2c_init.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    
    I2C_Init(MLX_I2C, &i2c_init);
    I2C_Cmd(MLX_I2C, ENABLE);                              // Enable the I2C peripheral

    /* Bus Recovery: Handle cases where SDA might be locked by a slave */
    timeout = 10000;
    while ((I2C_GetFlagStatus(MLX_I2C, I2C_FLAG_BUSY) == SET) && (timeout-- > 0)) {
        __NOP();
    }

    return MLX90614_OK;
}

/**
 * @brief Reads 16-bit raw data (LSB + MSB) from a specified temperature register.
 * @param reg_addr Register address (MLX90614_REG_TA or MLX90614_REG_TOBJ1).
 * @param[out] raw_temp Pointer to store the 16-bit raw temperature value.
 * @return MLX90614_Status_t Status of the read operation.
 */
MLX90614_Status_t MLX90614_ReadRawTemp(uint8_t reg_addr, uint16_t *raw_temp)
{
    uint8_t data_l, data_h;
    MLX90614_Status_t st;

    st = MLX_WaitBusFree();
    if (st != MLX90614_OK) return st;

    /* Write Phase: START + Slave Address (W) + Register Address */
    I2C_GenerateSTART(MLX_I2C, ENABLE);
    st = MLX_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT);
    if (st != MLX90614_OK) { I2C_GenerateSTOP(MLX_I2C, ENABLE); return st; }

    I2C_Send7bitAddress(MLX_I2C, MLX90614_ADDR_WRITE, I2C_Direction_Transmitter);
    st = MLX_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
    if (st != MLX90614_OK) { I2C_GenerateSTOP(MLX_I2C, ENABLE); return st; }

    I2C_SendData(MLX_I2C, reg_addr);
    st = MLX_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED);
    if (st != MLX90614_OK) { I2C_GenerateSTOP(MLX_I2C, ENABLE); return st; }

    /* Read Phase: Repeated START + Slave Address (R) */
    I2C_GenerateSTART(MLX_I2C, ENABLE);
    st = MLX_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT);
    if (st != MLX90614_OK) { I2C_GenerateSTOP(MLX_I2C, ENABLE); return st; }

    I2C_Send7bitAddress(MLX_I2C, MLX90614_ADDR_READ, I2C_Direction_Receiver);
    st = MLX_WaitEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);
    if (st != MLX90614_OK) { I2C_GenerateSTOP(MLX_I2C, ENABLE); return st; }

    /* Receive LSB */
    st = MLX_WaitFlagSet(I2C_FLAG_RXNE);
    if (st != MLX90614_OK) { I2C_GenerateSTOP(MLX_I2C, ENABLE); return st; }
    data_l = I2C_ReceiveData(MLX_I2C);

    /* Receive MSB */
    st = MLX_WaitFlagSet(I2C_FLAG_RXNE);
    if (st != MLX90614_OK) { I2C_GenerateSTOP(MLX_I2C, ENABLE); return st; }
    data_h = I2C_ReceiveData(MLX_I2C);

    /* Disable ACK before receiving the final byte (PEC) to generate a NACK */
    I2C_AcknowledgeConfig(MLX_I2C, DISABLE);
    
    /* Receive Packet Error Code (PEC) */
    st = MLX_WaitFlagSet(I2C_FLAG_RXNE);
    if (st != MLX90614_OK) {
        I2C_GenerateSTOP(MLX_I2C, ENABLE);
        I2C_AcknowledgeConfig(MLX_I2C, ENABLE);
        return st;
    }
    (void)I2C_ReceiveData(MLX_I2C); // PEC is received but currently discarded

    /* Transaction Complete: Generate STOP and Restore ACK for next time */
    I2C_GenerateSTOP(MLX_I2C, ENABLE);
    I2C_AcknowledgeConfig(MLX_I2C, ENABLE);

    *raw_temp = (uint16_t)(((uint16_t)data_h << 8) | data_l);
    return MLX90614_OK;
}

/**
 * @brief Reads Ambient Temperature in Celsius.
 * @param[out] temp Pointer to store the temperature in °C.
 * @return MLX90614_Status_t Status of the read operation.
 */
MLX90614_Status_t MLX90614_ReadAmbientTempC(float *temp)
{
    uint16_t raw;
    MLX90614_Status_t status;

    status = MLX90614_ReadRawTemp(MLX90614_REG_TA, &raw);
    if (status != MLX90614_OK) return status;

    /* Conversion Formula: Temp(K) = raw * 0.02, Temp(C) = Temp(K) - 273.15 */
    *temp = ((float)raw * 0.02f) - 273.15f;
    return MLX90614_OK;
}

/**
 * @brief Reads Object Temperature in Celsius.
 * @param[out] temp Pointer to store the temperature in °C.
 * @return MLX90614_Status_t Status of the read operation.
 */
MLX90614_Status_t MLX90614_ReadObjectTempC(float *temp)
{
    uint16_t raw;
    MLX90614_Status_t status;

    status = MLX90614_ReadRawTemp(MLX90614_REG_TOBJ1, &raw);
    if (status != MLX90614_OK) return status;

    /* Conversion Formula: Temp(K) = raw * 0.02, Temp(C) = Temp(K) - 273.15 */
    *temp = ((float)raw * 0.02f) - 273.15f;
    return MLX90614_OK;
}

MLX90614_Status_t MLX90614_ReadBodyTempC(float *temp)
{
    return MLX90614_ReadObjectTempC(temp);
}
///////////////////////////////////////////////////////////////
static void pf_enter_critical(void) 
{
#ifdef OS_SUPPORTING
    taskENTER_CRITICAL();
#else
    cpu_sr = __get_MSTATUS(); 
    __disable_irq();
#endif
}

static void pf_exit_critical(void) 
{
#ifdef OS_SUPPORTING
    taskEXIT_CRITICAL();
#else
    __set_MSTATUS(cpu_sr);
#endif
}
//实例化对象
static const mlx_iic_driver_instance_t  MLX_90614_iic_instance={
    #ifndef HARDWARE_IIC
    .pf_iic_init        = SHT40_SoftI2C_Init_Wrapper,
    .pf_iic_deinit      = SHT40_SoftI2C_DeInit,
    .pf_iic_send_ack    = SHT40_SoftI2C_SendAck,
    .pfiic_send_no_ack  = SHT40_SoftI2C_SendNoAck,
    .pf_iic_send_byte   = SHT40_SoftI2C_SendByte,
    .pf_iic_rec_byte    = SHT40_SoftI2C_ReadByte,
    .pf_enter_critical = pf_enter_critical,
    .pf_exit_critical  = pf_exit_critical
    #else
    .pf_iic_init        = NULL, // 硬件I2C由bsp_mlx_90614_driver_t的pf_init函数处理
    .pf_iic_deinit      = NULL, // 硬件I2C不需要单独的deinit函数
    .pf_iic_send_ack    = NULL, // 硬件I2C由底层库处理ACK
    .pfiic_send_no_ack  = NULL, // 硬件I2C由底层库处理NACK
    .pf_iic_send_byte   = NULL, // 硬件I2C由底层库处理数据发送
    .pf_iic_rec_byte    = NULL, // 硬件I2C由底层库处理数据接收
    .pf_enter_critical = pf_enter_critical,
    .pf_exit_critical  = pf_exit_critical
    #endif
};
//////////////////MLX90614 HIGH LEVEL FUNCTIONS//////////////////////
static void internal_delay_ms(bsp_mlx90614_driver_t *driver, uint32_t  ms)
{
#ifdef OS_SUPPORTING
    if (driver == NULL || driver->p_timebase_instance == NULL || driver->p_timebase_instance->pf_delay == NULL) {
        return; 
    }
    driver->p_timebase_instance->pf_delay(ms);
#endif
//不支持OS时,可以直接调用全局的delay函数,或者用户自行扩展接口
#ifndef OS_SUPPORTING
    Delay_Ms(ms);
#endif
}

static MLX90614_Status_t MLX90614_driver_init(void *context)
{
    bsp_mlx90614_driver_t *pf_driver_instance = (bsp_mlx90614_driver_t *)context;

    if (pf_driver_instance == NULL)
    {
#ifdef DEBUG_ENABLE
        printf("driver_instance is NULL\r\n");
#endif
        return MLX90614_ERROR;
    }
    //检查是否初始化
    if(IS_INITED)
    {
        return MLX90614_OK;
    }
    internal_delay_ms(pf_driver_instance, 100); // 确保设备上电稳定
#ifndef HARDWARE_IIC
    pf_driver_instance->p_iic_instance->pf_enter_critical();
    MLX90614_Status_t ret = pf_driver_instance->p_iic_instance->pf_iic_init(NULL);
    pf_driver_instance->p_iic_instance->pf_exit_critical();
    if(ret != MLX90614_OK) {
#ifdef DEBUG_ENABLE
        printf("MLX90614 initialization failed\r\n");
#endif
        return MLX90614_ERROR;
    }
    g_inited = MLX_INITED;
#ifdef DEBUG_ENABLE
    printf("MLX90614 initialized successfully\r\n");
#endif
    return MLX90614_OK;
#else

    MLX90614_Status_t ret = MLX90614_SoftI2C_Init();
 //   MLX90614_Status_t ret = pf_driver_instance->pf_init(context);
    if(ret != MLX90614_OK) {
#ifdef DEBUG_ENABLE
        printf("MLX90614 initialization failed\r\n");
#endif
        return MLX90614_ERROR;
    }
    g_inited = MLX_INITED;
#ifdef DEBUG_ENABLE
    printf("MLX90614 initialized successfully\r\n");
#endif
    return MLX90614_OK;
#endif
}

static MLX90614_Status_t MLX90614_driver_deinit(void *context)
{
    bsp_mlx90614_driver_t *pf_driver_instance = (bsp_mlx90614_driver_t *)context;

    if (pf_driver_instance == NULL)
    {
#ifdef DEBUG_ENABLE
        printf("driver_instance is NULL\r\n");  
#endif
        return MLX90614_ERROR;
    }
    if(!IS_INITED)
    {
        return MLX90614_OK; // Already deinitialized
    }else{
        g_inited = MLX_NOT_INITED;
        return MLX90614_OK;
    }   
}

 static MLX90614_Status_t MLX90614_ReadID(void *context, uint8_t * const id_buf, uint8_t buf_len)
{
    bsp_mlx90614_driver_t *driver = (bsp_mlx90614_driver_t *)context;

    if (NULL==driver || NULL == id_buf || buf_len == 0)
    {
#ifdef DEBUG_ENABLE
        printf("Invalid argument: driver or id_buf is NULL\r\n");
#endif
        return MLX90614_ERROR;
    }
    if(!IS_INITED)
    {
#ifdef DEBUG_ENABLE
        printf("MLX90614 not initialized\r\n");
#endif
        return MLX90614_ERROR;
    }
    // MLX90614 does not have a standard ID register, so this function can be implemented as needed, or return an error if not supported.
#ifdef DEBUG_ENABLE
    printf("MLX90614 does not support reading a unique ID\r\n");
#endif

    return MLX90614_OK;
}   

static MLX90614_Status_t read_data_raw(void *context, float * const temp)
{
    bsp_mlx90614_driver_t *driver = (bsp_mlx90614_driver_t *)context;

    if (driver == NULL || temp == NULL)
    {
#ifdef DEBUG_ENABLE
        printf("Invalid argument_raw: driver or temp is NULL\r\n");
#endif
        return MLX90614_ERROR;
    }
    if(!IS_INITED)
    {
#ifdef DEBUG_ENABLE
        printf("MLX90614 not initialized\r\n");
#endif
        return MLX90614_ERROR;
    }
    return MLX90614_ReadObjectTempC(temp);
}
static MLX90614_Status_t read_surface_temp(void *context, float * const temp)
{
    bsp_mlx90614_driver_t *driver = (bsp_mlx90614_driver_t *)context;

    if (driver == NULL || temp == NULL)
    {
#ifdef DEBUG_ENABLE
        printf("Invalid argument_surface_temp: driver or temp is NULL\r\n");
#endif
        return MLX90614_ERROR;
    }
    if(!IS_INITED)
    {
#ifdef DEBUG_ENABLE
        printf("MLX90614 not initialized\r\n"); 
#endif
        return MLX90614_ERROR;
    }
    return MLX90614_ReadAmbientTempC(temp);
}

static MLX90614_Status_t read_body_temp(void *context, float * const temp)
{
    bsp_mlx90614_driver_t *driver = (bsp_mlx90614_driver_t *)context;

    if (driver == NULL || temp == NULL)
    {
#ifdef DEBUG_ENABLE
        printf("Invalid argument: driver or temp is NULL\r\n");
#endif
        return MLX90614_ERROR;
    }
    if(!IS_INITED)
    {
#ifdef DEBUG_ENABLE
        printf("MLX90614 not initialized\r\n"); 
#endif
        return MLX90614_ERROR;
    }
    return MLX90614_ReadBodyTempC(temp);
}

static MLX90614_Status_t sleep(void *context)
{
    bsp_mlx90614_driver_t *driver = (bsp_mlx90614_driver_t *)context;

    if (driver == NULL)
    {
#ifdef DEBUG_ENABLE
        printf("Invalid argument: driver is NULL\r\n");
#endif
        return MLX90614_ERROR;
    }
    if(!IS_INITED)
    {
#ifdef DEBUG_ENABLE
        printf("MLX90614 not initialized\r\n");
#endif
        return MLX90614_ERROR;
    }
    return MLX90614_OK;
}

static MLX90614_Status_t wake(void *context)
{
    bsp_mlx90614_driver_t *driver = (bsp_mlx90614_driver_t *)context;

    if (driver == NULL)
    {
#ifdef DEBUG_ENABLE
        printf("Invalid argument: driver is NULL\r\n"); 
#endif
        return MLX90614_ERROR;
    }
    if(!IS_INITED)
    {
#ifdef DEBUG_ENABLE
        printf("MLX90614 not initialized\r\n");
#endif
        return MLX90614_ERROR;
    }
    return MLX90614_OK;
}
/****************public API*****************/
MLX90614_Status_t bsp_mlx_90614_inst(
            bsp_mlx90614_driver_t * const pf_driver,
             mlx_iic_driver_instance_t  *p_iic_instance,
    #ifdef OS_SUPPORTING
    mlx_os_timebase_interface_t  *p_os_timebase_instacne,
    #endif
    mlx_timebase_interface_t   *p_timebase_instance
)
{
    MLX90614_Status_t ret = MLX90614_OK;
if (pf_driver == NULL || p_timebase_instance == NULL || p_iic_instance == NULL)
{
#ifdef DEBUG_ENABLE
    printf("Invalid argument: pf_driver or its function pointers, p_timebase_instance, or p_iic_instance is NULL\r\n");
#endif
    return MLX90614_ERROR;
}
    pf_driver->p_iic_instance = p_iic_instance;
    pf_driver->p_timebase_instance = p_timebase_instance;
#ifdef OS_SUPPORTING
    pf_driver->p_os_timebase_instacne = p_os_timebase_instacne;
#endif
    pf_driver->pf_init = MLX90614_driver_init;
    pf_driver->pf_deinit = MLX90614_driver_deinit;
    pf_driver->pf_read_id = MLX90614_ReadID;
    pf_driver->pf_read_data_raw = read_data_raw;
    pf_driver->pf_read_surface_temp = read_surface_temp;
    pf_driver->pf_read_body_temp = read_body_temp;
    pf_driver->pf_sleep = sleep;
    pf_driver->pf_wake = wake;

    ret= pf_driver->pf_init(pf_driver);
    if (ret != MLX90614_OK) {
#ifdef DEBUG_ENABLE
        printf("MLX90614 driver initialization failed\r\n");    
#endif
        return MLX90614_ERROR;
    }
    return MLX90614_OK; 
}