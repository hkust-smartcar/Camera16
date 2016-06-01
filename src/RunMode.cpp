/*
 * RunMode.cpp
 *
 *  Created on: 26-03-2016
 *      Author: Kyle
 *      Adapted from code written by yungc
 *  Copyright © 2015-2016 HKUST SmartCar Team. All rights reserved.
 */

#include "../inc/RunMode.h"

#include <libsc/ab_encoder.h>
#include <libsc/dir_motor.h>
#include <libsc/encoder.h>
#include <libsc/trs_d05.h>
#include <libutil/misc.h>  //assist in protecting the hardware
#include <cstdint>

RunMode::RunMode() {
	//can initialize the variable here,
	motor->SetClockwise(false);
	maxMotorSpeed = 600;
	minMotorSpeed = 0;
	maxServoAngle = 1350;
	minServoAngle = 570;
	ideal_servo_degree = 900;
	ideal_motor_speed = 0;

	ServoErr = 0;
	ServoPrevErr = 0;

	MotorErr = 0;
	MotorPrev1Err = 0;
	MotorPrev2Err = 0;

}

RunMode::~RunMode() {

}

void RunMode::turningPID(const int8_t mid_line, float Kd, float T) {

//	float T = 0.3f; //TODO: find proper proportion and Kd
//	float Kd = 12.0f;

//Error=SetPoint-ProcessVariable
	ServoErr = mid_line - 39;

	/*-----Core dynamic PD formula-----*/
	//positional PD = T*(err in line)^2 * error +kd *(error-error_prev), try to let Kp be proportional to error squared
	ideal_servo_degree = uint16_t(
			900 + T * ServoErr * ServoErr * ServoErr
					+ Kd * (ServoErr - ServoPrevErr));

	//set servo accordingly
	servo->SetDegree(
			libutil::Clamp(minServoAngle, ideal_servo_degree, maxServoAngle));

	ServoPrevErr = ServoErr;
}

void RunMode::motorPID(int16_t ideal_encoder_count, float Kp, float Ki,
		float Kd) {

	encoder->Update();
	//Error=SetPoint-ProcessVariable
	MotorErr = ideal_encoder_count + encoder->GetCount(); //encoder count is negative, therefore the algebraic sum is plus

			/*-----Core PID formula-----*/
	// Incremental PID(n) = PID(n-1) + kp * (e(n)-e(n-1)) +kd *(e(n)-2e(n-1)+e(n-2)) + ki * e(n)
	ideal_motor_speed += Kp * (MotorErr - MotorPrev1Err) + Ki * MotorErr
			+ Kd * (MotorErr - 2 * MotorPrev1Err + MotorPrev2Err);
	motor->SetClockwise(ideal_motor_speed>0 ? false : true);

	motor->SetPower(
			libutil::Clamp(minMotorSpeed, (uint16_t)abs(ideal_motor_speed), maxMotorSpeed));

	MotorPrev2Err = MotorPrev1Err;
	MotorPrev1Err = MotorErr;
}

