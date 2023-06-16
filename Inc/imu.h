/*
 * imu.h
 *
 *  Created on: May 19, 2023
 *      Author: MARIUS TALI
 *              GAVIN GODDARD
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#include <stdint.h>
#include "stm32f4xx.h"

/// IMU Driver Header File for Function Prototypes.

#ifdef __cplusplus
 extern "C" {
#endif


typedef struct imu_structs {
	uint16_t            dev_addr; // I2C Device Address
	I2C_HandleTypeDef*  hi2c;     // &hi2c(n)
} imu_structs_t;


void startup(imu_structs_t imu_objs);
void calibrate(imu_structs_t imu_objs);
int16_t temperature(imu_structs_t imu_objs);
int16_t yaw(imu_structs_t imu_objs);
int16_t roll(imu_structs_t imu_objs);
int16_t pitch(imu_structs_t imu_objs);

#ifdef __cplusplus
}
#endif

#endif /* INC_IMU_H_ */
