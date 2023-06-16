/*
 * Encoder.cpp
 *
 *  Created on: May 22, 2023
 *      Author: MARIUS TALI
 *              GAVIN GODDARD
 */


//------------INCLUDES---------
#include <Encoder.h>
#include <stdio.h>
#include <stm32f4xx_hal_def.h>
#include <usbd_cdc_if.h>
//-------------\/--------------

/// Encoder class with all required functions.
///
/// This class will allow the user to interact with the encoder on the Pololu ungeared motor.
/// The encoder is a 48 PPR encoder.



/// Start the Encoder
///
/// This function turns on the timer channel associated with encoder.
/// @param enc This is the encoder declaration in Encoder.h.
/// @note The encoder will take up a full timer. The current PCB sets Timer 2.
void start(enc_struct_t enc_objs)
{
	HAL_TIM_Encoder_Start(enc_objs.htim, TIM_CHANNEL_ALL);
}
//-----------------------------------------\/-------------------------------------------

/// Read the Encoder
///
/// This function gets the current tick value of the timer which is ticked every encoder pass.
/// @param enc This is the encoder declaration in Encoder.h.
/// @return The current number of encoder ticks.
/// @warning There hasn't been any overflow protection built in as a 32 bit integer should give us
/// over 30 hrs of run time at our max duty cycle without zeroing before overflow. This should be
/// more than enough for most applications.
int32_t position(enc_struct_t enc_objs)
{
	int32_t position = enc_objs.TIM->CNT;
	return position;
}
//-----------------------------------------\/-------------------------------------------

/// Zero the Encoder
///
/// This function sets the count value of the timer to zero.
/// @param enc This is the encoder declaration in Encoder.h.
void zero(enc_struct_t enc_objs)
{
	enc_objs.TIM->CNT = 0;
}
//-----------------------------------------\/-------------------------------------------
