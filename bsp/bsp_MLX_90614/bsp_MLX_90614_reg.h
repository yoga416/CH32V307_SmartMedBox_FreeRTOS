#ifndef _BSP_MLX_90614_REG_H
#define _BSP_MLX_90614_REG_H


/** * @name I2C Device Address Definitions
 * @{ 
 */
#define MLX90614_ADDR_7BIT        0x5A /**< Default 7-bit I2C slave address */
#define MLX90614_ADDR_WRITE       ((MLX90614_ADDR_7BIT << 1) | 0x00U) /**< 8-bit Write address (0xB4) */
#define MLX90614_ADDR_READ        ((MLX90614_ADDR_7BIT << 1) | 0x01U) /**< 8-bit Read address (0xB5) */
/** @} */

/** * @name RAM Register Addresses
 * @{ 
 */
#define MLX90614_REG_TA           0x06 /**< Ambient temperature register address (Ta) */
#define MLX90614_REG_TOBJ1        0x07 /**< Object 1 temperature register address (Tobj1) */
/** @} */


// 其他相关定义
//温湿度阈值
#define MLX90614_TEMP_MIN_C       10.0f /**< Minimum valid temperature in Celsius */
#define MLX90614_TEMP_MAX_C       80.0f /**< Maximum valid temperature in Celsius */

/* ==================================================================== */
/* =============== 软件 I2C 引脚配置 (替换为 PB6 和 PB7) ================ */
/* ==================================================================== */
#define MLX_SCL_PORT   GPIOB
#define MLX_SCL_PIN    GPIO_Pin_6
#define MLX_SDA_PORT   GPIOB
#define MLX_SDA_PIN    GPIO_Pin_7

#define MLX_RCC_CMD    RCC_APB2PeriphClockCmd
#define MLX_RCC_PORT   RCC_APB2Periph_GPIOB  // 都在GPIOB，时钟不用改

#endif