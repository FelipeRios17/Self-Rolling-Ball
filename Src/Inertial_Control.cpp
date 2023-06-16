/*
 * Inertial_Control.cpp
 *
 *  Created on: Jun 13, 2023
 *      Author: MARIUS TALI
 *              GAVIN GODDARD
 */

//------------INCLUDES---------
#include <Inertial_Control.h>
#include <Encoder.h>
#include <Motor_Driver.h>
#include <stdio.h>
#include <stm32f4xx_hal_def.h>
#include <usbd_cdc_if.h>
#include <math.h>
#include "usb_device.h"
#include "usbd_cdc_if.h"
//-------------\/--------------

/// Inertial wheel driver class with all required functions.
///
/// This class will allow the user to interact with the inertial disk on
/// the robot and allow for quick bursts of spin.


//---------VARIABLES---------
int32_t prev_tick = 0;
int32_t curr_tick = 0;
int32_t prev_pos = 0;
int32_t curr_pos = 0;
uint32_t first_time = 1;
uint32_t first_tick = 0;

uint32_t sign_flag = 0;
uint32_t turn_flag = 1;

int32_t act_vel = 0;
int32_t tar_vel = 0;
int32_t err_vel = 0;
int32_t duty = 0;

uint32_t cnt = 0;

uint8_t uart_msgs[60] = {0};
//-----------\/--------------


/// Start Inertial Wheel
///
/// This function starts up the inertial wheel at a slow rate so as to avoid introducing
/// too much angular momentum at system startup.
/// @param enc This is the encoder declaration in Encoder.h.
/// @param mot This is the motor declaration in Motor_Driver.h.
/// @param direction This is the flag that is sent from the remote letting the robot know
/// which direction to spin the wheel.
/// @note This is not run on a timer, but rather uses HAL_GetTick() to find run its control.
/// @attention This code was still in development at time of project completion. When run, the
/// robot would start spinning in the direction of startup, then begin spinning in reverse at
/// break which ultimately led to the bot returning to its original position. This still needs
/// tuning.
void slow_start(enc_struct_t enc, mot_struct_t mot, uint8_t direction)
{
	// 2400 ticks/s is 100% duty
	if (first_time == 1)
	{
		zero(enc);
		prev_pos = 0;
		first_time = 0;
		turn_flag = 1;
		first_tick = HAL_GetTick();
		prev_tick = first_tick;
	}
	curr_tick = HAL_GetTick();
	curr_pos = position(enc);
	act_vel = (curr_pos - prev_pos)*1000/(curr_tick - prev_tick);
	// Target velocity if based on 1-exp(-0.5t) curve
	tar_vel = (1 - exp((double)(-0.5*(curr_tick - first_tick)))) * 2400;
	err_vel = tar_vel - act_vel;

	duty = (err_vel * 1) / 24; // Kp = 1

	if (direction == 2)
	{
		mot.duty = duty;
	}
	else
	{
		mot.duty = -duty;
	}

	set_duty(mot);
	sprintf((char*)uart_msgs,"direction: %d\r\n\n", direction);
	CDC_Transmit_FS((uint8_t*)uart_msgs, strlen((char*)uart_msgs));

	prev_tick = curr_tick;
	prev_pos = curr_pos;

	if (duty < 0)
	{
		sign_flag = 1; // Negative
	}

}
//-----------------------------------------\/-------------------------------------------


/// Brake the Inertial Wheel
///
/// This function reverses the current duty cycle by setting new duty cycle to maximum but
/// opposite. It continues at this duty until the velocity of the wheel changes sign at which
/// time the duty will be set to zero. Also has a cnt function to set duty to zero if remote
/// ever becomes unresponsive during operation.
/// @param enc This is the encoder declaration in Encoder.h.
/// @param mot This is the motor declaration in Motor_Driver.h.
/// @note This is not run on a timer, but rather uses HAL_GetTick() to find run its control.
uint32_t fast_break(enc_struct_t enc, mot_struct_t mot)
{
	if (sign_flag == 1) // Negative duty
	{
		curr_tick = HAL_GetTick();
		curr_pos = position(enc);
		act_vel = (curr_pos - prev_pos)*1000/(curr_tick - prev_tick);

		mot.duty = 100;
		if (act_vel >= 0)
		{
			mot.duty = 0;
			first_time = 1;
			sign_flag = 0;
			turn_flag = 0;
		}
		else
		{
			prev_tick = curr_tick;
			prev_pos = curr_pos;
			cnt++;
		}
		set_duty(mot);
	}
	else
	{
		curr_tick = HAL_GetTick();
		curr_pos = position(enc);
		act_vel = (curr_pos - prev_pos)*1000/(curr_tick - prev_tick);

		mot.duty = -100;

		if (act_vel <= 0)
		{
			mot.duty = 0;
			first_time = 1;
			sign_flag = 0;
			turn_flag = 0;
			act_vel = 0;
		}
		else
		{
			prev_tick = curr_tick;
			prev_pos = curr_pos;
			cnt++;
		}
		set_duty(mot);
	}

	if (cnt >= 15)
	{
		HAL_GPIO_WritePin(GPIOC, led_Pin, GPIO_PIN_RESET);
		mot.duty = 0;
		set_duty(mot);
		first_time = 1;
		sign_flag = 0;
		turn_flag = 0;
		cnt  = 0;
	}

	return turn_flag;
}
//-----------------------------------------\/-------------------------------------------




