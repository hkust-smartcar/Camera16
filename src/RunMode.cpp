/*
 * RunMode.cpp
 *
 *  Created on: 26-03-2016
 *      Author: Kyle, Bling Bling
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
				1270), minServoAngle(600), maxMotorSpeed(600), minMotorSpeed(0), ServoErr(
				0), ServoPrevErr(0), ideal_servo_degree(SERVO_MID), MotorErr(0), MotorPrev1Err(
				0), ideal_motor_speed(0) {
}

RunMode::~RunMode() {
}

void RunMode::turningPID(int8_t mid_line, const VarSet& m_varset,
		const bool IsCross) {

//Error=SetPoint-ProcessVariable
	if (IsCross)
		mid_line = 1.4 * mid_line - 15.6;
	ServoErr = mid_line - 39;

	/*-----Core dynamic PD formula-----*/
	//positional PD = T * error^2 +kd *(error-error_prev)
	if (ServoErr < 0) {
		ideal_servo_degree = uint16_t(
				SERVO_MID + m_varset.l_Kp * abs(ServoErr) * ServoErr
						+ m_varset.l_Kd * (ServoErr - ServoPrevErr));
	} else {
		ideal_servo_degree = uint16_t(
				SERVO_MID + m_varset.r_Kp * abs(ServoErr) * ServoErr
						+ m_varset.r_Kd * (ServoErr - ServoPrevErr));
	}

	//set servo accordingly
	servo->SetDegree(
			libutil::Clamp(minServoAngle, int16_t(ideal_servo_degree),
					maxServoAngle));

	ServoPrevErr = ServoErr;
}

void RunMode::motorPID(const VarSet& m_varset) {

	encoder->Update();
	//Error=SetPoint-ProcessVariable
	encodercount = -encoder->GetCount(); //record raw encoder count for comparison
//	encodercount = encodercount - m_beta * (encodercount - real_encodercount); //LPF
	if (abs(encodercount) > 5000)
		encodercount = m_varset.ideal_encoder_count;
	MotorErr = int16_t(
			(m_varset.ideal_encoder_count
					- (m_varset.ideal_encoder_count == 0 ?
							0 : m_varset.KDec * ServoErr * ServoErr))
					- encodercount);

	/*-----Core PI formula-----*/
	// Incremental PID(n) = PID(n-1) + kp * (e(n)-e(n-1)) +kd *(e(n)-2e(n-1)+e(n-2)) + ki * e(n)
	ideal_motor_speed += m_varset.Kp * (MotorErr - MotorPrev1Err)
			+ m_varset.Ki * MotorErr;
	motor->SetClockwise(ideal_motor_speed > 0 ? false : true);

	motor->SetPower(
			libutil::Clamp(minMotorSpeed, uint16_t(abs(ideal_motor_speed)),
					maxMotorSpeed));

	MotorPrev1Err = MotorErr;
}

VarSet RunMode::SelectVarSet(void) {
	//speed, servo l_Kp, l_Kd, r_Kp, r_Kd motor Kp, Ki, offset, KDec, Crossroad Mode, allow stop
	const VarSet myVS1_p = { 0, 1.4f, 75, 1.4f, 75, 0.9f, 0.01f, 8, 1.0f,
			VarSet::CrossroadMode::kLazy, true ,53}; //left vacant for tuning
	const VarSet myVS1_r = { 0, 1.4f, 75, 1.4f, 75, 0.9f, 0.01f, 8, 0.9f,
			VarSet::CrossroadMode::kLazy, false,49 };
	const VarSet myVS1_s = { 2400, 1.4f, 75, 1.4f, 75, 0.9f, 0.01f, 8, 0.9f,
			VarSet::CrossroadMode::kLazy, true ,49};
	const VarSet myVS2 = { 2500, 1.4f,75, 1.4f, 75, 0.9f, 0.01f, 8, 1.0f,
			VarSet::CrossroadMode::kLazy, true ,53}; //confirmed
	const VarSet myVS3 = { 2600, 1.4f, 75, 1.4f, 75, 0.9f, 0.01f, 8, 1.0f,
			VarSet::CrossroadMode::kLazy, true ,52}; //confirmed
	const VarSet myVS4 = { 2000, 1.36f, 38, 1.36f, 38, 0.45f, 0.03f, 8, 0.9,
			VarSet::CrossroadMode::kLazy, true ,49}; //basically confirmed
	const VarSet myVS5 = { 2100, 1.41f, 48, 1.36f, 38, 0.45f, 0.03f, 8, 0.9,
			VarSet::CrossroadMode::kLazy, true ,49}; //almost confirmed
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
			PrintVarSet(m_selected,libsc::Lcd::kWhite);
			break;
		case 1:
			m_selected = myVS1_r;
			PrintVarSet(m_selected,libsc::Lcd::kWhite);
			break;
		case 2:
			m_selected = myVS1_s;
			PrintVarSet(m_selected,libsc::Lcd::kWhite);
			break;
		case 3:
			m_selected = myVS2;
			PrintVarSet(m_selected,libsc::Lcd::kCyan);
			break;
		case 4:
			m_selected = myVS3;
			PrintVarSet(m_selected,libsc::Lcd::kGreen);
			break;
		case 5:
			m_selected = myVS4;
			PrintVarSet(m_selected,libsc::Lcd::kYellow);
			break;
		case 6:
			m_selected = myVS5;
			PrintVarSet(m_selected,libsc::Lcd::kRed);
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

inline void RunMode::PrintVarSet(const VarSet m_varset,const uint16_t m_color) {

	printvalue(0, 60, 40, 20, m_varset.ideal_encoder_count, m_color);

	switch(m_varset.xMode){
	case VarSet::CrossroadMode::kLazy:
		printvalue(0, 80, 128, 20, "~~Lazy~~", m_color);
		break;
	case VarSet::CrossroadMode::kAllWhite:
		printvalue(0, 80, 128, 20, "~~AllWhite~~", m_color);
		break;
	case VarSet::CrossroadMode::kOutwards:
		printvalue(0, 80, 128, 20, "~~Outwards~~", m_color);
	}

	if (m_varset.allow_stop) {
		printvalue(0, 100, 128, 20, "~True!~", m_color);
	} else {
		printvalue(0, 100, 128, 20, "~False~", m_color);
	}
}

