/*
 * driverdriver.h
 *
 *  Created on: May 19, 2023
 *      Author: MARIUS TALI
 *              GAVIN GODDARD
 */

#ifndef __DRIVERDRIVER_H
#define __DRIVERDRIVER_H

#include <sys/_stdint.h>
#include <stdint.h> // Allows use of standard integer types
#include <stdio.h>  // Allows the use of printf()
#include "stm32f4xx.h"

#ifdef __cplusplus
 extern "C" {
#endif

 /// IMU Control Header File for Function Prototypes.

typedef struct imu_struct{
	int32_t             pitch; // IMU Euler Pitch
	int32_t             roll;  // IMU Euler Roll
	int32_t             yaw;   // IMU Euler Heading/Yaw
} imu_struct_t;


void data_capture(imu_struct_t main_imu, imu_struct_t remote_imu);
void data_saturation();
void target_calc();
void orientation_calc();
int32_t new_duty1();
int32_t new_duty2();
int32_t new_duty3();

#ifdef __cplusplus
}
#endif

#endif // __DRIVERDRIVER_H
