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
		varset_index(0), selecting_varset(true), encodercount(0), maxServoAngle(
				1200), minServoAngle(570), maxMotorSpeed(400), minMotorSpeed(0), ServoErr(
				0), ServoPrevErr(0), ideal_servo_degree(900), MotorErr(0), MotorPrev1Err(
				0), MotorPrev2Err(0), ideal_motor_speed(0) {
}

RunMode::~RunMode() {
}

void RunMode::turningPID(const int8_t mid_line, const float Kp, const float Kd,
		const uint8_t thres, const float straight_Kp, const float straight_Kd) { //TODO: different PID for different intervals

//Error=SetPoint-ProcessVariable
	ServoErr = mid_line - 39;

	/*-----Core dynamic PD formula-----*/
	//positional PD = T * error^2 +kd *(error-error_prev)
	if (abs(ServoErr) < thres)
		ideal_servo_degree = uint16_t(
				900 + straight_Kp * ServoErr
						+ straight_Kd * (ServoErr - ServoPrevErr));
	else
		ideal_servo_degree = uint16_t(
				900 + Kp * abs(ServoErr) * ServoErr
						+ Kd * (ServoErr - ServoPrevErr));

	//set servo accordingly
	servo->SetDegree(
			libutil::Clamp(minServoAngle, int16_t(ideal_servo_degree), maxServoAngle));

	ServoPrevErr = ServoErr;
}

void RunMode::motorPID(const int16_t ideal_encoder_count, const float Kp,
		const float Ki, const float Kd, const float KDec) {

	encoder->Update();
	//Error=SetPoint-ProcessVariable
	encodercount = -encoder->GetCount(); //record raw encoder count for comparison
//	encodercount = encodercount - m_beta * (encodercount - real_encodercount); //LPF
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
			libutil::Clamp(minMotorSpeed, uint16_t( abs(ideal_motor_speed)),
					maxMotorSpeed));

	MotorPrev2Err = MotorPrev1Err;
	MotorPrev1Err = MotorErr;
}

VarSet RunMode::SelectVarSet(void) {
	//speed, servo Kp, Kd, motor Kp, Ki, Kd, offset, KDec
	VarSet myVS1_p = { 0, 1.35f, 21.0f, 0.2f, 0.0205f, 0, 8, 0,
			VarSet::PlannerMode::kRoot }; //left vacant for tuning
	VarSet myVS1_r = { 0, 1.6f, 2.6f, 0.2f, 0.0205f, 0, 8, 0,
			VarSet::PlannerMode::kProportional };
	VarSet myVS1_s = { 0, 1.6f, 2.6f, 0.2f, 0.0205f, 0, 8, 0,
			VarSet::PlannerMode::kSquared };
	VarSet myVS2 = { 1800, 1.6f, 1.85f, 0.36f, 0.03f, 0.65f, 8, 0,
			VarSet::PlannerMode::kProportional }; //strongly confirmed
	VarSet myVS3 = { 1900, 1.65f, 1.85f, 0.36f, 0.03f, 0.65f, 8, 8,
			VarSet::PlannerMode::kProportional }; //confirmed
	VarSet myVS4 = { 2000, 1.8f, 2.0f, 0.36f, 0.03f, 0.65f, 8, 8.5,
			VarSet::PlannerMode::kProportional }; //basically confirmed
	VarSet myVS5 = { 2000, 1.77f, 5.0f, 0.36f, 0.03f, 0.65f, 8, 9,
			VarSet::PlannerMode::kProportional }; //not for this camera angle
	VarSet m_selected = myVS1_p;
	printvalue(0, 0, 128, 20, "HKUST Camera", libsc::Lcd::kGray); //some welcome messages
	printvalue(0, 40, 128, 20, "Select Speed:", libsc::Lcd::kCyan);
	printvalue(30, 20, 60, 20, "cV", libsc::Lcd::kWhite);
	for (;;) { //loop infinitely until VarSet selected
		libbase::k60::Watchdog::Refresh(); //remember to treat your doggy well
#ifndef TESTSERVO
		if (varset_index > 7)
			varset_index = 6; //if uint8_t overflowed, causing index==100+, set it right
		if (varset_index == 7)
			varset_index = 0; //if reaches the end, loop back to 0
		if (!selecting_varset)
			break; //if selected by pressing select on joystick, break the pathetic infinite loop

		switch (varset_index) { //print speed according to corresponding VarSet
		case 0:
			m_selected = myVS1_p;
			printvalue(0, 60, 40, 20, m_selected.ideal_encoder_count,
					libsc::Lcd::kGreen);
			break;
		case 1:
			m_selected = myVS1_r;
			printvalue(0, 60, 40, 20, m_selected.ideal_encoder_count,
					libsc::Lcd::kGreen);
			break;
		case 2:
			m_selected = myVS1_s;
			printvalue(0, 60, 40, 20, m_selected.ideal_encoder_count,
					libsc::Lcd::kYellow);
			break;
		case 3:
			m_selected = myVS2;
			printvalue(0, 60, 40, 20, m_selected.ideal_encoder_count,
					libsc::Lcd::kRed);
			break;
		case 4:
			m_selected = myVS3;
			printvalue(0, 60, 40, 20, m_selected.ideal_encoder_count,
					libsc::Lcd::kPurple);
			break;
		case 5:
			m_selected = myVS4;
			printvalue(0, 60, 40, 20, m_selected.ideal_encoder_count,
					libsc::Lcd::kPurple);
			break;
		case 6:
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
