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
#include "car.h"

class RunMode: public Car
{
	// tips for choosing datatype, dont use too large( it will slow down your program)
	//and dont use too small (otherwise underflow occur)
	// you may check 8bit,16bit,and 32bit int & uint range fromthe link:
	//			http://www.societyofrobots.com/member_tutorials/book/export/html/341
	//or just google that if u are not sure
public:
	RunMode();
	~RunMode();

	//positional PID = kp *error +kd *(error_prev - error), try to let Kp be proportional to error squared
	void turningPID (int8_t const mid_line,float Kp,float T) override;

	// Incremental PID(n) = PID(n-1) + kp * (e(n)-e(n-1)) +kd *(e(n)-2e(n-1)+e(n-2)) + ki * e(n)
	// which means previous PID, two of the previous errors should be cached
	void motorPID (int16_t ideal_encoder_count,float,float,float) override;

	//--------------------------variable below---------------------------//
	//to access the public variable, you can use (obj_name).(var_name) to access

	int16_t ideal_servo_degree, ideal_motor_speed;
	int16_t MotorErr,ServoErr;// put to public to facilitate tuning


private: //yes, I add these variable as private, because they are not important
	// Moreover, variable can be declare in header(.h), and define in either header(.h) or source(.cpp)

	int16_t maxServoAngle, minServoAngle;		// give a maximun& minimun angle for servo to turn
	int16_t maxMotorSpeed, minMotorSpeed; // give a maximun& minimun PWM for motor to run

//	int16_t ServoErr;
	int16_t ServoPrevErr;

//	int16_t MotorErr;
	int16_t MotorPrev1Err;
	int16_t MotorPrev2Err;

};
