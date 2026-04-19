
#ifndef __SHT40_H__
#define __SHT40_H__

/********************************INCLUDE FILES********************************/
#include "sht40_reg.h"
#include <stdint.h>

/********************************* DEFINE ************************************/
/* 取消注释以启用相应的宏 */
#define  OS_SUPPORTING  /* OS_SUPPORTING / NO_OS_SUPPORTING */
#define  software_I2C   /* HARDWARE_I2C / software_I2C */

/* Return values from SHT40 functions */
typedef enum {
    SHT40_OK          = 0,       /**< Operation successful */
    SHT40_ERROR       = 1,       /**< Generic error */
    SHT40_ERRORSOURCE = 2        /**< Communication or sensor error */
} SHT40_Status_t;

typedef struct {
    SHT40_Status_t (*pf_iic_init)       (void *);    // I2C initialization function pointer
    SHT40_Status_t (*pf_iic_deinit)     (void *);    // I2C deinitialization function pointer
    SHT40_Status_t (*pf_iic_start)      (void *);    // I2C start condition function pointer
    SHT40_Status_t (*pf_iic_stop)       (void *);    // I2C stop condition function pointer
    SHT40_Status_t (*pf_iic_wait_ack)   (void *);    // I2C wait for ACK function pointer
    SHT40_Status_t (*pf_iic_send_ack)   (void *);    // I2C send ACK function pointer
    SHT40_Status_t (*pf_iic_send_no_ack)(void *);    // I2C send NACK function pointer
    SHT40_Status_t (*pf_iic_send_byte)  (void *, const uint8_t); // I2C send byte function pointer
    SHT40_Status_t (*pf_iic_read_byte)  (void *, uint8_t * const); // I2C read byte function pointer

    // 进入临界区和退出临界区的函数指针
    void (*pf_enter_critical)(void); 
    void (*pf_exit_critical) (void); 

} iic_driver_interface_t;

typedef struct {
    uint32_t (*pf_get_tick_count)(void);    // 获取系统当前 tick 计数的函数指针
    void     (*pf_delay_ms)(uint32_t);      // 修复：必须添加裸机延时函数指针，否则 .c 中调用会报错
} timebase_interface_t;

#ifdef OS_SUPPORTING
typedef struct {
    void (*pf_os_delay_ms)(const uint32_t); // 操作系统毫秒级延时函数指针
} os_timebase_interface_t;
#endif 

typedef struct {
    iic_driver_interface_t      *p_iic_interface; 
    timebase_interface_t        *p_timebase_interface; 
#ifdef OS_SUPPORTING
    os_timebase_interface_t     *p_os_timebase_interface; 
#endif

    SHT40_Status_t (*pf_init)               (void *const); 
    SHT40_Status_t (*pf_deinit)             (void *const);
    SHT40_Status_t (*pf_read_id)            (void *const, uint64_t *const); // 修复：必须增加 64位指针用于回传 ID
    SHT40_Status_t (*pf_read)               (void *const, float * const temp, float * const humi); 
    SHT40_Status_t (*pf_read_humidity)      (void *const, float * const humi); 
    SHT40_Status_t (*pfsleep)               (void *const); 
    SHT40_Status_t (*pf_wakeup)             (void *const); 
} bsp_sht40_driver_t;

SHT40_Status_t SHT40_inst(
    bsp_sht40_driver_t       *const  p_sht40_instance, 
    iic_driver_interface_t   *const  p_iic_interface, 
#ifdef OS_SUPPORTING
    os_timebase_interface_t  *const  p_os_timebase_interface, 
#endif
    timebase_interface_t     *const  p_timebase_interface 
); 

#endif /* end of __SHT40_H__ */