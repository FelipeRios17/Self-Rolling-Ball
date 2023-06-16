/*
 * imu.cpp
 *
 *  Created on: May 19, 2023
 *      Author: MARIUS TALI
 *              GAVIN GODDARD
 */

//------------INCLUDES---------
#include <imu.h>
#include <math.h>
#include <stdio.h>
#include <stm32f4xx_hal_def.h>
#include <stm32f4xx_hal_i2c.h>
#include <string.h>
#include <sys/_stdint.h>
#include <usbd_cdc_if.h>
//-------------\/--------------

/// IMU class with all required functions.
///
/// This class will allow the user to interact with by sending/receiving data
/// from the BNO055 IMU on both the robot and remote.

//---------IMU REGISTER ADDRESSES---------
static const uint8_t OPR_MODE_REG = 0x3D;
static const uint8_t CALIB_REG = 0x35;
static const uint8_t TEMP_REG = 0x34;
static const uint8_t EULX_REG = 0x1A;
static const uint8_t EULY_REG = 0x1C;
static const uint8_t EULZ_REG = 0x1E;
//------------------\/--------------------

HAL_StatusTypeDef ret; // Error Handling
uint8_t calib_done = 0;


/// Enable the IMU
///
/// This function writes the operating mode to the IMU for initialization.
/// @param imu This is the imu declaration in imu.h.
/// @note This is currently set to the NDOF Fusion Mode
void startup(imu_structs_t imu_objs)
{
	uint8_t set = 0x0C;
	// NDOF Fusion mode for MAG, ACCEL, and GYRO Absolute readings
	HAL_I2C_Mem_Write(imu_objs.hi2c, imu_objs.dev_addr, OPR_MODE_REG, 1, &set, 1, 1000);
}
//-----------------------------------------\/-------------------------------------------

/// Calibrate the Magnetometer
///
/// This function will check the magnetometer calibration register
/// for a finished calibrating flag and alert the user.
/// @param imu This is the imu declaration in imu.h.
/// @attention In order to calibrate, once the robot is powered on, it must be waved in
/// a figure-eight motion for 5-10 seconds. The red status LED will flash when calibrated.
void calibrate(imu_structs_t imu_objs)
{
	uint8_t reg = 0;
	while (calib_done == 0)
	{
		HAL_I2C_Mem_Read(imu_objs.hi2c, imu_objs.dev_addr, CALIB_REG, 1, &reg, 1, 1000);
		HAL_Delay(50);
		reg = reg & 3; // Bottom 2 bits correspond to mag calibration
		if (reg == 3)
		{
			calib_done = 1;
			HAL_GPIO_WritePin(GPIOC, led_Pin, GPIO_PIN_RESET);
			HAL_Delay(500);
			HAL_GPIO_WritePin(GPIOC, led_Pin, GPIO_PIN_SET);
		}
	}
}
//-----------------------------------------\/-------------------------------------------

/// Find Temperature
///
/// This function finds the temperature of the IMU using the accel temperature sensor.
/// @param imu This is the imu declaration in imu.h.
/// @note This is not used in our project.
int16_t temperature(imu_structs_t imu_objs)
{
	uint8_t buf[12] = {0};
	int16_t val;
	int16_t temp_c;

	ret = HAL_I2C_Mem_Read(imu_objs.hi2c, imu_objs.dev_addr, TEMP_REG, 1, buf, 1, 1000);
		if (ret != HAL_OK) // If HAL transmit failed, show error message
		{
			strcpy((char*)buf, "Error RX \r\n");
		}
		else
		{
			val = (int16_t)buf[0];
			if (val > 0x7F) // Check if num > than max positive signed number
			{
				val |= 0xF00; // If yes, sign extend a nibble of 1's in front of num
			}
			temp_c = val;
			sprintf((char*)buf,"%d C \r\n", temp_c);
		}
	    //CDC_Transmit_FS((uint8_t*)buf,strlen((char*)buf));
	    return temp_c;
}
//---------------------------------------\/----------------------------------------------

/// Read IMU Yaw
///
/// This function finds the current value of IMU Yaw in Euler Angles.
/// @param imu This is the imu declaration in imu.h.
/// @return The IMU yaw value.
int16_t yaw(imu_structs_t imu_objs)
{
	uint8_t buf[2] = {0};
	uint8_t buf1[15] = {0};

	HAL_I2C_Mem_Read(imu_objs.hi2c, imu_objs.dev_addr, EULX_REG, 1, buf, 2, 1000);
	int16_t eulx = round((buf[1] << 8 | buf[0])/16); // Shift MSB left and fill in LSB

	sprintf((char*)buf1,"YAW: %d deg \r\n", eulx);
	//CDC_Transmit_FS((uint8_t*)buf1, strlen((char*)buf1));

	return eulx;
}
//------------------------------------\/-------------------------------------------

/// Read IMU Roll
///
/// This function finds the current value of IMU Roll in Euler Angles.
/// @param imu This is the imu declaration in imu.h.
/// @return The IMU roll value.
int16_t roll(imu_structs_t imu_objs)
{
	uint8_t buf[2] = {0};
	uint8_t buf1[15] = {0};
	int16_t euly;

	HAL_I2C_Mem_Read(imu_objs.hi2c, imu_objs.dev_addr, EULY_REG, 1, buf, 2, 1000);
	if (buf[1] > 0x7F)
	{
		int16_t eul = (buf[1] << 8 | buf[0])|0xF0000;
		euly = round(eul/16);
	}
	else
	{
		euly = round((buf[1] << 8 | buf[0])/16);
	}

	sprintf((char*)buf1,"ROLL: %d deg \r\n", euly);
	//CDC_Transmit_FS((uint8_t*)buf1, strlen((char*)buf1));

	return euly;
}
//-------------------------------------\/------------------------------------------

/// Read IMU Pitch
///
/// This function finds the current value of IMU Pitch in Euler Angles.
/// @param imu This is the pitch declaration in imu.h.
/// @return The IMU pitch value.
int16_t pitch(imu_structs_t imu_objs)
{
	uint8_t buf[2] = {0};
	uint8_t buf1[350] = {0};
	int16_t eulz;

	HAL_I2C_Mem_Read(imu_objs.hi2c, imu_objs.dev_addr, EULZ_REG, 1, buf, 2, 1000);
	if (buf[1] > 0x7F)
	{
		int16_t eul = (buf[1] << 8 | buf[0])|0xF0000;
		eulz = round(eul/16);
	}
	else
	{
		eulz = round((buf[1] << 8 | buf[0])/16);
	}
	sprintf((char*)buf1,"PITCH: %d deg \r\n", eulz);
	//CDC_Transmit_FS((uint8_t*)buf1, strlen((char*)buf1));

	return eulz;
}
//--------------------------------------\/-----------------------------------------


