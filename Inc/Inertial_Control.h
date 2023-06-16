/*
 * Inertial_Control.h
 *
 *  Created on: Jun 13, 2023
 *      Author: MARIUS TALI
 *              GAVIN GODDARD
 */

#ifndef INC_INERTIAL_CONTROL_H_
#define INC_INERTIAL_CONTROL_H_

/// Inertial Motor Control Header File for Function Prototypes.
#include <stdint.h>
#include "stm32f4xx.h"
#include "Motor_Driver.h"
#include "Encoder.h"

#ifdef __cplusplus
 extern "C" {
#endif


void slow_start(enc_struct_t enc_struct, mot_struct_t mot_struct, uint8_t direction);
uint32_t fast_break(enc_struct_t enc_struct, mot_struct_t mot_struct);


#ifdef __cplusplus
}
#endif

#endif /* INC_INERTIAL_CONTROL_H_ */
