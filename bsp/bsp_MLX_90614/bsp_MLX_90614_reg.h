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

/** * @name GPIO Pin Definitions
 * @{ 
 */
#define MLX_SDA_PIN               GPIO_Pin_11 /**< SDA Pin for software I2C */
#define MLX_SCL_PIN               GPIO_Pin_10 /**< SCL Pin for software I2C */
/** @} */

/* I2C Configuration for MLX90614 */
#define MLX_I2C                   I2C2
#define MLX_I2C_TIMEOUT           60000U /**< Timeout threshold for I2C operations */




#endif