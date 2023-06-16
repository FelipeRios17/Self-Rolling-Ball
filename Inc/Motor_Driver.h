/*
 * Charlie_Driver.h
 *
 *  Created on: Apr 20, 2023
 *      Author: MARIUS TALI
 *              GAVIN GODDARD
 */

#ifndef INC_MOTOR_DRIVER_H_
#define INC_MOTOR_DRIVER_H_

/// Motor Driver Header File for Function Prototypes.

#ifdef __cplusplus
 extern "C" {
#endif


#include <stdint.h>
#include "stm32f4xx.h"

typedef struct mot_struct {
	int32_t             duty;     // Duty cycle percentage -100 -> 100
	uint32_t            ar;       // Timer auto reload value
	uint32_t            chan_a;   // TIM_CHANNEL_na
	uint32_t            chan_b;   // TIM_CHANNEL_nb
	uint16_t            en_pin;   // Enable pin
	GPIO_TypeDef*       gpio;     // GPIO(n)
	TIM_HandleTypeDef*  htim;     // &htim(n)
} mot_struct_t;

void enable_motor(mot_struct_t mot_objs);
void disable_motor(mot_struct_t mot_objs);
void set_duty(mot_struct_t mot_objs);

#ifdef __cplusplus
}
#endif

#endif /* INC_MOTOR_DRIVER_H_ */
