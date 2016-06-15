/*
 * RunMode.cpp
 *
 *  Created on: 26-03-2016
 *      Author: Kyle
 *      Adapted from code written by yungc
 *  Copyright © 2015-2016 HKUST SmartCar Team. All rights reserved.
 */

#include "../inc/RunMode.h"

#include <libbase/k60/watchdog.h>
#include <libsc/system.h>
#include <libsc/ab_encoder.h>
#include <libsc/dir_motor.h>
#include <libsc/encoder.h>
#include <libsc/trs_d05.h>
#include <libutil/misc.h>  //assist in protecting the hardware
#include <libsc/lcd.h>
#include <stdint.h>
#include <cstdlib>

RunMode::RunMode() :
		varset_index(0), selecting_varset(true), real_encodercount(0), encodercount(
				0), maxServoAngle(1210), minServoAngle(590), maxMotorSpeed(400), minMotorSpeed(
				0), ServoErr(0), ServoPrevErr(0), ideal_servo_degree(900), MotorErr(
				0), MotorPrev1Err(0), MotorPrev2Err(0), ideal_motor_speed(0) {
}

RunMode::~RunMode() {
}

void RunMode::turningPID(const int8_t mid_line, const float Kd, float T,
		const uint8_t thres) {

//Error=SetPoint-ProcessVariable
	ServoErr = mid_line - 39;

	/*-----Core dynamic PD formula-----*/
	//positional PD = T * error^2 +kd *(error-error_prev)
	if (abs(ServoErr) < thres)
		ideal_servo_degree = uint16_t(
				900 + T * ServoErr + Kd * (ServoErr - ServoPrevErr));
	else
		ideal_servo_degree = uint16_t(
				900 + T * abs(ServoErr) * ServoErr
						+ Kd * (ServoErr - ServoPrevErr));

	//set servo accordingly
	servo->SetDegree(
			libutil::Clamp(minServoAngle, ideal_servo_degree, maxServoAngle));

	ServoPrevErr = ServoErr;
}

void RunMode::motorPID(const int16_t ideal_encoder_count, const float Kp,
		const float Ki, const float Kd, const float m_beta, const float KDec) {

	encoder->Update();
	//Error=SetPoint-ProcessVariable
	real_encodercount = -encoder->GetCount(); //record raw encoder count for comparison
	encodercount = encodercount - m_beta * (encodercount - real_encodercount); //LPF
	if (abs(encodercount) > 5000)
		encodercount = ideal_encoder_count;
	MotorErr = int16_t(
			(ideal_encoder_count
					- (ideal_encoder_count == 0 ? 0 : KDec * abs(ServoErr)))
					- encodercount);

	/*-----Core PID formula-----*/
	// Incremental PID(n) = PID(n-1) + kp * (e(n)-e(n-1)) +kd *(e(n)-2e(n-1)+e(n-2)) + ki * e(n)
	ideal_motor_speed += Kp * (MotorErr - MotorPrev1Err) + Ki * MotorErr
			+ Kd * (MotorErr - 2 * MotorPrev1Err + MotorPrev2Err);
	motor->SetClockwise(ideal_motor_speed > 0 ? false : true);

	motor->SetPower(
			libutil::Clamp(minMotorSpeed, (uint16_t) abs(ideal_motor_speed),
					maxMotorSpeed));

	MotorPrev2Err = MotorPrev1Err;
	MotorPrev1Err = MotorErr;
}

VarSet RunMode::SelectVarSet(void) {
	//speed, servo Kp, Kd, motor Kp, Ki, Kd, β,offset, KDec
	VarSet myVS1 = { 0, 3.0f, 0, 0.3f, 0.03f, 0.7f, 0.4f, 8, 0 }; //left vacant for tuning
	VarSet myVS2 = { 2000, 1.447f, 1.9f, 0.36f, 0.03f, 0.7f, 0.4f, 8, 9 }; //working fine
	VarSet myVS3 = { 2100, 1.44f, 1.87f, 0.36f, 0.03f, 0.7f, 0.4f, 8, 11 }; //testing
	VarSet myVS4 = { 850, 1.5f, 0.47f, 1.0f, 0.08f, 1.4f, 1.0f, 8, 12 }; //not sure
	VarSet myVS5 = { 1250, 2.5f, 0.2f, 0.22f, 0.08f, 2.0f, 1.0f, 8, 12 }; //havn't tested
	VarSet m_selected = myVS1;
	printvalue(0, 0, 128, 20, "HKUST Camera", libsc::Lcd::kGray); //some welcome messages
	printvalue(0, 40, 128, 20, "Select Speed:", libsc::Lcd::kCyan);
	printvalue(30, 20, 60, 20, "cV", libsc::Lcd::kWhite);
	for (;;) { //loop infinitely until VarSet selected
		libbase::k60::Watchdog::Refresh(); //remember to treat your doggy well
#ifndef TESTSERVO
		if (varset_index > 5)
			varset_index = 4; //if uint8_t overflowed, causing index==100+, set it right
		if (varset_index == 5)
			varset_index = 0; //if reaches the end, loop back to 0
		if (!selecting_varset)
			break; //if selected by pressing select on joystick, break the pathetic infinite loop

		switch (varset_index) { //print speed according to corresponding VarSet
		case 0:
			m_selected = myVS1;
			printvalue(0, 60, 40, 20, m_selected.ideal_encoder_count,
					libsc::Lcd::kGreen);
			break;
		case 1:
			m_selected = myVS2;
			printvalue(0, 60, 40, 20, m_selected.ideal_encoder_count,
					libsc::Lcd::kGreen);
			break;
		case 2:
			m_selected = myVS3;
			printvalue(0, 60, 40, 20, m_selected.ideal_encoder_count,
					libsc::Lcd::kYellow);
			break;
		case 3:
			m_selected = myVS4;
			printvalue(0, 60, 40, 20, m_selected.ideal_encoder_count,
					libsc::Lcd::kRed);
			break;
		case 4:
			m_selected = myVS5;
			printvalue(0, 60, 40, 20, m_selected.ideal_encoder_count,
					libsc::Lcd::kPurple);
			break;
		}
#else
		printvalue(deg,libsc::Lcd::kWhite);
		servo->SetDegree(deg);
#endif
		printvalue(0, 20, 30, 20, int16_t(batt->GetVoltage() * 100),
				libsc::Lcd::kWhite);
		libsc::System::DelayMs(20); //don't overload the mcu before image processing even begin
	}
	return m_selected;
}
