/*
 * Encoder.h
 *
 *  Created on: May 22, 2023
 *      Author: gavin
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>
#include "stm32f4xx.h"

/// Encoder Driver Header File for Function Prototypes.


typedef struct enc_struct {
	TIM_HandleTypeDef*  htim;     // &htim(n)
	TIM_TypeDef*        TIM;      // TIMx
} enc_struct_t;


void start(enc_struct_t enc_struct);
int32_t position(enc_struct_t enc_struct);
void zero(enc_struct_t enc_struct);

#ifdef __cplusplus
}
#endif

#endif /* INC_ENCODER_H_ */
