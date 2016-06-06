/*
 * RunMode.cpp
 *
 *  Created on: 26-03-2016
 *      Author: this
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

RunMode::RunMode() {
	//can initialize the variable here,
	motor->SetClockwise(false);
	maxMotorSpeed = 600;
	minMotorSpeed = 0;
	maxServoAngle = 1240;
	minServoAngle = 540;
	ideal_servo_degree = 900;
	ideal_motor_speed = 0;

	ServoErr = 0;
	ServoPrevErr = 0;

	MotorErr = 0;
	MotorPrev1Err = 0;
	MotorPrev2Err = 0;

	varset_index = 0;
	selecting_varset = true;

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
			900 + T * abs(ServoErr) * ServoErr
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
	motor->SetClockwise(ideal_motor_speed > 0 ? false : true);

	motor->SetPower(
			libutil::Clamp(minMotorSpeed, (uint16_t) abs(ideal_motor_speed),
					maxMotorSpeed));

	MotorPrev2Err = MotorPrev1Err;
	MotorPrev1Err = MotorErr;
}

VarSet RunMode::SelectVarSet(void) {
	VarSet myVS1 = { 0, 2.5f, 0.2f, 0.21f, 0.08f, 0.35f, 5, 59 }; //0
	VarSet myVS2 = { 850, 2.5f, 0.2f, 0.21f, 0.02f, 0.35f, 5, 59 }; //850
	VarSet myVS3 = { 1050, 2.5f, 0.2f, 0.21f, 0.02f, 0.35f, 5, 59 }; //1050
	VarSet myVS4 = { 1150, 2.5f, 0.2f, 0.21f, 0.02f, 0.35f, 5, 59 }; //1150
	VarSet myVS5 = { 1250, 2.5f, 0.2f, 0.21f, 0.02f, 0.35f, 5, 59 }; //1250
	VarSet m_selected = myVS1;
	printvalue(0, 0, 128, 20, "HKUST Camera", libsc::Lcd::kGray); //some welcome messages
	printvalue(0, 40, 128, 20, "Select Speed:", libsc::Lcd::kCyan);
	printvalue(30,20,60,20,"cV",libsc::Lcd::kWhite);
	for (;;) { //loop infinitely until VarSet selected
		libbase::k60::Watchdog::Refresh(); //remember to treat your doggy well

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
		//		this.printvalue(deg,Lcd::kWhite);
		//		this.GetServo().SetDegree(deg);
		printvalue(0, 20, 30, 20, int16_t(batt->GetVoltage() * 100),
				libsc::Lcd::kWhite);
		libsc::System::DelayMs(20); //don't overload the mcu before image processing even begin
	}
	return m_selected;
}
