/**
 * @file bsp_MLX_90614.h
 * @brief MLX90614 non-contact infrared temperature sensor interface definitions.
 *
 * This driver implements a software I2C (bit-banging) interface to communicate 
 * with the MLX90614 sensor on the CH32V30x platform.
 */

#ifndef _BSP_MLX_90614_H_
#define _BSP_MLX_90614_H_

#include "ch32v30x.h"
#include "bsp_MLX_90614_reg.h"

#define HARDWARE_IIC  //software/HARDWARE_IIC
#define DEBUG_ENABLE   //DEBUG_ENABLE/NOT_DEBUG_ENABLE
#define OS_SUPPORTING //OS_SUPPORTING  /NOT_OS_SUPPORTING

typedef enum {
    MLX90614_OK            = 0, /**< Operation successful */
    MLX90614_ERROR         = 1, /**< Generic communication error */
    MLX90614_INVALID_PARAM = 2, /**< Invalid parameter provided to function */
    MLX90614_TIMEOUT       = 3  /**< I2C bus timeout occurred */
} MLX90614_Status_t;

#ifndef HARDWARE_IIC
//定义iic的结构体
typedef struct{
MLX90614_Status_t (*pf_iic_init)        (void *);
MLX90614_Status_t (*pf_iic_deinit)      (void *);
MLX90614_Status_t (*pf_iic_start)       (void *);
MLX90614_Status_t (*pf_iic_stop)        (void *);
MLX90614_Status_t (*pf_iic_send_ack)    (void *);
MLX90614_Status_t (*pfiic_send_no_ack)  (void *);
MLX90614_Status_t (*pf_iic_wait_ack)    (void *);
MLX90614_Status_t (*pf_iic_send_byte)   (void *,const uint8_t );
MLX90614_Status_t (*pf_iic_rec_byte)    (void *,uint8_t * const);
 // 进入临界区和退出临界区的函数指针
    void (*pf_enter_critical)(void); 
    void (*pf_exit_critical) (void);
}mlx_iic_driver_instance_t;
#else
typedef struct{
MLX90614_Status_t (*pf_iic_init)        (void *);
MLX90614_Status_t (*pf_iic_deinit)      (void *);
MLX90614_Status_t (*pf_iic_send_ack)    (void *);
MLX90614_Status_t (*pfiic_send_no_ack)  (void *);
MLX90614_Status_t (*pf_iic_send_byte)   (void *,const uint8_t );
MLX90614_Status_t (*pf_iic_rec_byte)    (void *,uint8_t * const);
 // 进入临界区和退出临界区的函数指针
    void (*pf_enter_critical)(void); 
    void (*pf_exit_critical) (void);
}mlx_iic_driver_instance_t;
#endif
//case timebase_instance
typedef struct {
    uint32_t (*pf_get_count)(void *);
    uint32_t (*pf_get_ms)(void *);
    uint32_t (*pf_get_us)(void *);
    void (*pf_delay)(uint32_t);
}mlx_timebase_interface_t;

//os_interface
#ifdef OS_SUPPORTING
typedef struct {
    void (*pf_os_delay)(const uint32_t);
}mlx_os_timebase_interface_t;
#endif

typedef struct {
    mlx_iic_driver_instance_t  *p_iic_instance;
    mlx_timebase_interface_t   *p_timebase_instance;
    #ifdef OS_SUPPORTING
    mlx_os_timebase_interface_t  *p_os_timebase_instacne;
    #endif

    MLX90614_Status_t (*pf_init)(void *);
    MLX90614_Status_t (*pf_deinit)(void *);
    MLX90614_Status_t (*pf_read_id)(void *, uint8_t * const id_buf, uint8_t buf_len);
    MLX90614_Status_t (*pf_read_data_raw)(void * const, float * const temp);
    MLX90614_Status_t (*pf_read_surface_temp)(void * const, float * const temp);
     MLX90614_Status_t (*pf_read_body_temp)(void * const, float * const temp);
    MLX90614_Status_t (*pf_sleep)(void *);
    MLX90614_Status_t (*pf_wake)(void *);

}bsp_mlx90614_driver_t;

MLX90614_Status_t bsp_mlx_90614_inst(
            bsp_mlx90614_driver_t * const pf_driver,
             mlx_iic_driver_instance_t  *p_iic_instance,
    #ifdef OS_SUPPORTING
    mlx_os_timebase_interface_t  *p_os_timebase_instacne,
    #endif
    mlx_timebase_interface_t   *p_timebase_instance
);


MLX90614_Status_t MLX90614_SoftI2C_Init(void);

#endif /* _BSP_MLX_90614_H_ */