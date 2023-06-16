/*
 * driverdriver.cpp
 *
 *  Created on: May 19, 2023
 *      Authors: MARIUS TALI
 *               GAVIN GODDARD
 */

//------------INCLUDES---------
#include <driverdriver.h>
#include <sys/_stdint.h>
#include <usbd_cdc_if.h>
#include <cmath>
#include <cstring>
//-------------\/--------------

//---------VARIABLES---------
int32_t main_pitch = 0;
int32_t main_roll = 0;
int32_t main_yaw = 0;
int32_t remote_pitch = 0;
int32_t remote_roll = 0;
int32_t remote_yaw = 0;

int32_t m_mag = 0;
int32_t r_mag = 0;
int32_t true_target = 0;
int32_t main_target = 0;
int32_t relative_target = 0; 

int32_t duty_1 = 0; 
int32_t duty_2 = 0; 
int32_t duty_3 = 0; 
//-----------\/--------------

/// IMU Driver class with all required functions.
///
/// This class will allow the user to interact the IMU on the robot with
/// the IMU on the remote to allow for quick and intuitive feedback.


/// Capture IMU Data
///
/// This function captures the data from both the main and remote IMUs to
/// prevent inconsistent results due to fluctuating values.
/// @param main_imu This is the imu struct defined in driverdriver.h for the main IMU
/// @param main_imu This is the imu struct defined in driverdriver.h for the remote IMU
/// @note imu_struct_t is separately defined from imu_structs_t as seen in imu.cpp. The
/// structure passed here contains the yaw, pitch, and roll data that was collected using
/// the imu_structs_t structure and class in imu.cpp.
void data_capture(imu_struct_t main_imu, imu_struct_t remote_imu)
{
    main_pitch = main_imu.pitch;
    main_roll = main_imu.roll;
    main_yaw = main_imu.yaw;
    remote_pitch = remote_imu.pitch;
    remote_roll = remote_imu.roll;
    remote_yaw = remote_imu.yaw;
}

/// Saturate IMU Data
///
/// This function saturates first the remote IMU data to a range of +-45 deg from
/// a neutral position. This is done for ease of use by the user. The IMU on the
/// robot is saturated to a range of +-60 deg to prevent over rotation and flipping.
/// @note This function should only be called after data_capture()
void data_saturation()
{
    if (remote_pitch <= 135 && remote_pitch >= 0)  // -135 -> -180 = 180 -> 135
    {
        remote_pitch = 135;
    }
    else if (remote_pitch >= -135 && remote_pitch < 0)
    {
        remote_pitch = -135;
    }

    if (remote_pitch > 0)
    {
    	remote_pitch = remote_pitch - 180;
    }
    else if (remote_pitch < 0)
    {
    	remote_pitch = remote_pitch + 180;
    }

    if (remote_roll >= 45) // -45 -> 45
    {
        remote_roll = 45;
    }
    else if (remote_roll <= -45)
    {
        remote_roll = -45;   
    }
    
    //---------------------------------------------------------------------------
    if (main_pitch <= 120 && main_pitch >= 0)  // -120 -> -180 = 180 -> 120
	{
		main_pitch = 120;
	}
	else if (main_pitch >= -120 && main_pitch < 0)
	{
		main_pitch = -120;
	}

	if (main_pitch > 0)
	{
		main_pitch = main_pitch - 180;
	}
	else if (main_pitch < 0)
	{
		main_pitch = main_pitch + 180;
	}

	if (main_roll >= 60) // -60 -> 60
	{
		main_roll = 60;
	}
	else if (main_roll <= -60)
	{
		main_roll = -60;
	}

}


/// Find Target Direction
///
/// This function calculates the relative direction of the double angle tilt of the robot using the
/// remote IMU. This is done through the use of the atan() funciton with roll and pitch measurements.
/// @note This function should only be called after data_saturation()
void target_calc()
{
    if (remote_roll != 0 && remote_pitch == 0)
    {
        true_target = 1.5708 * 1000; // Handles divide by zero case
    }
    else if (remote_pitch >= 0)
    {
        true_target = atan((double)(remote_roll/remote_pitch)) * 1000; // Multiplied by 1000 to avoid float
    }
    else if (remote_pitch < 0)
    {
        true_target = (atan((double)(remote_roll/remote_pitch)) + 3.14159) * 1000;
    }
    else
    {
        true_target = 0;
    }
    // Compensating for Robot heading difference from true North
    true_target = true_target - ( remote_yaw + main_yaw + 90 )*3.14159*1000/180;
    // |v| = √(x^2 + y^2) Divided by an expected maximum to achieve unit length
    r_mag = sqrt(remote_pitch*remote_pitch+remote_roll*remote_roll)/45 * 1000;
    // Due to the current square inputs, rounding out the edges
    if (r_mag >= 1000)
    {
        r_mag = 1000;
    }
    // Provide a large "neutral zone" for the user with zero magnitude.
    if (r_mag <= 100)
    {
    	r_mag = 0;
    }
}


/// Find Orientation
///
/// This function calculates the relative direction of the double angle tilt of the robot using the
/// robot IMU. This is done through the use of the atan() funciton with roll and pitch measurements.
/// It is used to help control the robot tilt during operation.
/// @note This function should only be called after target_calc()
void orientation_calc()
{
	if (main_roll != 0 && main_pitch == 0)
	{
		main_target = 1.5708 * 1000;
	}
	else if (main_pitch >= 0)
	{
		main_target = atan((double)(main_roll/main_pitch)) * 1000;
	}
	else if (main_pitch < 0)
	{
		main_target = (atan((double)(main_roll/main_pitch)) + 3.14159) * 1000;
	}
	else
	{
		main_target = 0;
	}

	main_target = main_target - 45*3.14159*1000/180;
	m_mag = sqrt(main_pitch*main_pitch+main_roll*main_roll)/60 * 1000;
	if (m_mag >= 1000)
	{
		m_mag = 1000;
	}
}


/// Set Motor 2 Duty Cycle
///
/// This function calculates the duty cycle to be sent to motor 2 of the robot.
/// An intermediate duty is calculated to control the robot rotation within the ball.
/// @return The duty cycle used to set the motor.
/// @note This function should only be called after target_calc() and orientation_calc()
/// @attention It is important to select the correct motor/lead direction for this duty.
int32_t new_duty1()
{
    uint8_t buf[20] = {0};
    // Find magnitude of robot vector in line with the remote vector
    int32_t duty_intm1 = r_mag - m_mag*cos((double)((main_target - true_target)/1000));
    if (duty_intm1 < 0)
	{
    	duty_intm1 = 0;
	}
	duty_1 = sin((double)true_target/1000)*duty_intm1/10;
	// Create deadzone where motors can't overcome stiction.
	if (duty_1 <= 20 && duty_1 > 0)
	{
		duty_1 = 0;
	}
	else if (duty_1 >= -20 && duty_1 < 0)
	{
		duty_1 = 0;
	}
	sprintf((char*)buf,"Duty 1: %ld \r\n", duty_1);
	CDC_Transmit_FS((uint8_t*)buf, strlen((char*)buf));

	return duty_1;
}

/// Set Motor 3 Duty Cycle
///
/// This function calculates the duty cycle to be sent to motor 3 of the robot.
/// An intermediate duty is calculated to control the robot rotation within the ball.
/// @return The duty cycle used to set the motor.
/// @note This function should only be called after target_calc() and orientation_calc()
/// @attention It is important to select the correct motor/lead direction for this duty.
int32_t new_duty2()
{
	uint8_t buf2[20] = {0};
	int32_t duty_intm2 = r_mag - m_mag*cos((double)((main_target - true_target)/1000));
	duty_2 = sin(((double)true_target/1000)-2.0944)*duty_intm2/10; // Subtract pi/3 for 3 wheels
    if (duty_intm2 < 0)
	{
    	duty_intm2 = 0;
	}
	if (duty_2 <= 20 && duty_2 > 0)
	{
		duty_2 = 0;
	}
	else if (duty_2 >= -20 && duty_2 < 0)
	{
		duty_2 = 0;
	}
    sprintf((char*)buf2,"Duty 2: %ld \r\n", duty_2);
    //CDC_Transmit_FS((uint8_t*)buf2, strlen((char*)buf2));

    return duty_2;
}

/// Set Motor 4 Duty Cycle
///
/// This function calculates the duty cycle to be sent to motor 4 of the robot.
/// An intermediate duty is calculated to control the robot rotation within the ball.
/// @return The duty cycle used to set the motor.
/// @note This function should only be called after target_calc() and orientation_calc()
/// @attention It is important to select the correct motor/lead direction for this duty.
int32_t new_duty3()
{
	uint8_t buf3[20] = {0};
	int32_t duty_intm3 = r_mag - m_mag*cos((double)((main_target - true_target)/1000));
	duty_3 = sin(((double)true_target)/1000-4.1888)*duty_intm3/10;
    if (duty_intm3 < 0)
	{
    	duty_intm3 = 0;
	}
	if (duty_3 <= 20 && duty_3 > 0)
	{
		duty_3 = 0;
	}
	else if (duty_3 >= -20 && duty_3 < 0)
	{
		duty_3 = 0;
	}
    sprintf((char*)buf3,"Duty 3: %ld \r\n\n", duty_3);
    //CDC_Transmit_FS((uint8_t*)buf3, strlen((char*)buf3));

    return duty_3;
}
