#ifndef _BSP_MLX_90614_PORT_H_
#define _BSP_MLX_90614_PORT_H_

#include "bsp_MLX_90614.h"
#include "bsp_MLX_90614_handler.h"
#include "Middle_ring_buffer.h"

MLX90614_Status_t MLX90614_Port_InitDriver(bsp_mlx90614_driver_t *driver);
MLX90614_Status_t MLX90614_Port_BindHandlerInput(th_handler_input_instance_t *input,
												bsp_mlx90614_driver_t *driver);

void MLX90614_Port_OnDataReady(float *surface_temp, float *body_temp);
void MLX90614_Handler_start(void);

extern th_handler_os_instance_t g_mlx90614_freertos_if;

#endif /* _BSP_MLX_90614_PORT_H_ */
