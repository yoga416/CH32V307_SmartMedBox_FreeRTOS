#ifndef _SHT40_REG_H_
#define _SHT40_REG_H_

/* Software I2C Pin Definitions (GPIO simulation of I2C protocol) */
#define SHT40_SOFT_I2C_SCL_PORT      GPIOC
#define SHT40_SOFT_I2C_SCL_PIN       GPIO_Pin_0
#define SHT40_SOFT_I2C_SDA_PORT      GPIOA
#define SHT40_SOFT_I2C_SDA_PIN       GPIO_Pin_1

/* Timing Parameters */
#define SHT40_SOFT_I2C_DELAY_CYCLES  1000   /**< Delay cycles for I2C frequency control */
#define SHT40_SOFT_I2C_TIMEOUT       10000  /**< Software I2C timeout (loop iterations) */



#define SHT40_MEASURE_CMD           0xFD /**< High precision measurement command (no clock stretching) */
/** @} */

/**
 * @brief Structure to hold sensor measurement results.
 */
typedef struct {
    float temperature_c;    /**< Temperature in degrees Celsius (°C) */
    float humidity_rh;      /**< Relative Humidity in percentage (%RH) */
} SHT40_Data_t;





#endif