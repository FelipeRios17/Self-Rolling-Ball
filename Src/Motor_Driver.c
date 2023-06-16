/*
 * Charlie_Driver.c
 *
 *  Created on: Apr 20, 2023
 *      Author: MARIUS TALI
 *              GAVIN GODDARD
 */

#include <Motor_Driver.h>

/// Motor driver with all required functions.
///
/// This class will allow the user to interact with the motor driver ICs on the PCB

/// Enable the Motor
///
/// This function sends a high signal to the enable pin on the PCB and starts the timer
/// channels used for each motor.
/// @param mot This is the motor declaration in Motor_Driver.h.
void enable_motor(mot_struct_t mot_objs) {
	HAL_TIM_PWM_Start(mot_objs.htim,mot_objs.chan_a);
	HAL_TIM_PWM_Start(mot_objs.htim,mot_objs.chan_b);
	HAL_GPIO_WritePin(mot_objs.gpio, mot_objs.en_pin, GPIO_PIN_SET);
}

/// Disable the Motor
///
/// This function sends a low signal to the enable pin on the PCB and disables the timer
/// channels used for each motor to save power.
/// @param mot This is the motor declaration in Motor_Driver.h.
void disable_motor(mot_struct_t mot_objs) {
	HAL_TIM_PWM_Stop(mot_objs.htim,mot_objs.chan_a);
	HAL_TIM_PWM_Stop(mot_objs.htim,mot_objs.chan_b);
	HAL_GPIO_WritePin(mot_objs.gpio, mot_objs.en_pin, GPIO_PIN_RESET);
}

/// Set Motor Duty Cycle
///
/// This function sets the compare value of each timer channel for PWM
/// generation. This can handle both negative and positive PWM.
/// @param mot This is the motor declaration in Motor_Driver.h.
void set_duty(mot_struct_t mot_objs) {
	if(mot_objs.duty < 0) {
		__HAL_TIM_SET_COMPARE(mot_objs.htim,mot_objs.chan_a,0);
		__HAL_TIM_SET_COMPARE(mot_objs.htim,mot_objs.chan_b,-mot_objs.duty*mot_objs.ar/100);
		HAL_Delay(10);
	}
	/* duty[%]*AR[ticks]/100[%] represents a number between 0 and AR ticks.
	 * This is based on using a frequency of 20kHz to spin the motor with a
	 * specific prescaler. If duty comes in as a percentage, divide by 100
	 * and multiply by AR to get range of values in ticks.
	 */
	else {
		__HAL_TIM_SET_COMPARE(mot_objs.htim,mot_objs.chan_a,mot_objs.duty*mot_objs.ar/100);
		__HAL_TIM_SET_COMPARE(mot_objs.htim,mot_objs.chan_b,0);
		HAL_Delay(10);
	}
}
