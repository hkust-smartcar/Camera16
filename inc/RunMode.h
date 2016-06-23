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
#define USE_LCD
#define USE_PGRAPHER
//#define ADJUST_CAM
//#define TESTSERVO

#include <cstdint>

#include "car.h"

struct VarSet {
	enum struct PlannerMode {
		kRoot = 0, kProportional, kSquared
	};

	int16_t ideal_encoder_count;
	/*-----servo-----*/
	float T; //kp
	float K; //kd

	/*-----motor-----*/
	float Kp;
	float Ki;
	float Kd;

	/*-----other processing variables-----*/
	int8_t offset;
	float KDec; //deceleration constant
	PlannerMode mode;
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
	void turningPID(int8_t const real_mid_line, const float Kp, const float Kd,
			uint8_t thres, const float straight_Kp, const float straight_Kd)
					override;

	// Incremental PID(n) = PID(n-1) + kp * (e(n)-e(n-1)) +kd *(e(n)-2e(n-1)+e(n-2)) + ki * e(n)
	// which means previous PID, two of the previous errors should be cached
	void motorPID(const int16_t ideal_encoder_count, const float, const float,
			const float, const float) override;

	VarSet SelectVarSet(void);

	//--------------------------variable below---------------------------//
	//to access the public variable, you can use (obj_name).(var_name) to access
	uint8_t varset_index;
	bool selecting_varset;
#ifdef TESTSERVO
	uint16_t deg=900;
#endif
#ifdef ADJUST_CAM
	uint8_t m_brightness=0x00;
	uint8_t m_contrast=0x40;
#endif
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
