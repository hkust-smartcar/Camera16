/*
 * RunMode.h
 *
 *  Created on: 28-3-2016
 *      Author: Kyle
 *      Adapted from code written by yungc
 *  Copyright © 2015-2016 HKUST SmartCar Team. All rights reserved.
 *
 *  Handle PID controllers for motor and servo, keeps the history encoder data
 */
#pragma once
#define USE_PGRAPHER
//#define TESTSERVO

#include <cstdint>

#include "car.h"

struct VarSet {
	int16_t ideal_encoder_count;
	/*-----servo-----*/
	float T; //kp
	float K; //kd

	/*-----motor-----*/
	float Kp;
	float Ki;
	float Kd;
	float motor_beta; //beta for motor's LPF

	/*-----other processing variables-----*/
	int8_t offset;
	uint8_t KDec; //deceleration constant
};

class RunMode: public Car {
	// tips for choosing datatype, dont use too large( it will slow down your program)
	//and dont use too small (otherwise underflow occur)
	// you may check 8bit,16bit,and 32bit int & uint range fromthe link:
	//			http://www.societyofrobots.com/member_tutorials/book/export/html/341
	//or just google that if u are not sure
public:
	RunMode();
	~RunMode();

	//positional PID = kp *error +kd *(error_prev - error), try to let Kp be proportional to error squared
	void turningPID(int8_t const real_mid_line, const float Kp, const float T)
			override;

	// Incremental PID(n) = PID(n-1) + kp * (e(n)-e(n-1)) +kd *(e(n)-2e(n-1)+e(n-2)) + ki * e(n)
	// which means previous PID, two of the previous errors should be cached
	void motorPID(const int16_t ideal_encoder_count, const float, const float,
			const float, const float,const uint8_t) override;

	VarSet SelectVarSet(void);

	//--------------------------variable below---------------------------//
	//to access the public variable, you can use (obj_name).(var_name) to access
	uint8_t varset_index;
	bool selecting_varset;
#ifdef TESTSERVO
	uint16_t deg=900;
#endif
	int32_t real_encodercount;
	int32_t encodercount;

private:
	//yes, I add these variable as private, because they are not important
	// Moreover, variable can be declare in header(.h), and define in either header(.h) or source(.cpp)

	const int16_t maxServoAngle, minServoAngle;	// give a maximun& minimun angle for servo to turn
	const uint16_t maxMotorSpeed, minMotorSpeed; // give a maximun& minimun PWM for motor to run

	int16_t ServoErr;
	int16_t ServoPrevErr;
	int16_t ideal_servo_degree;

	int16_t MotorErr;
	int16_t MotorPrev1Err;
	int16_t MotorPrev2Err;
	int16_t ideal_motor_speed;

};
