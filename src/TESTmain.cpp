/*
 * main.cpp
 *
 * Author: Kyle
 * Adapted from code written by Peter
 * Copyright © 2015-2016 HKUST SmartCar Team. All rights reserved.
 * Refer to LICENSE for details
 */

#include <cassert>
#include <cstring>
#include <libsc/system.h>
#include <stdint.h>
//#include "pVarManager.h"
//#include "car.h"
#include "RunMode.h"
#include <ImageProcess.h>
#include <Planner.h>
#include <libutil/looper.h>

using namespace libsc;

using namespace libbase::k60;

using namespace libutil;

int main(void) {

//code for ploting graph for a equation of y = mx +c, where y and x are encoder counting or motor PWM
//uncomment for usage
	/*
	 //tune encoder here
	 //to uncomment this code, comment all pVarManager object
	 JyMcuBt106::Config config;
	 config.id = 0;
	 config.baud_rate = libbase::k60::Uart::Config::BaudRate::k115200;
	 config.rx_irq_threshold = 2;
	 JyMcuBt106 fuck(config);
	 char *PWM_buffer = new char[120]{0};
	 float encoder_counting = 0;
	 int motor_speed =0;
	 while(1){
	 motor_speed += 1;
	 Run.motor_control(motor_speed,true);
	 Run.update_encoder();
	 System::DelayMs(30);
	 Run.update_encoder();

	 encoder_counting = Run.get_encoder_count();
	 int n = sprintf(PWM_buffer,"%d %d \n",(int)motor_speed,(int)encoder_counting);
	 fuck.SendBuffer((Byte*)PWM_buffer,n);
	 memset(PWM_buffer,0,n);
	 if (motor_speed > 500) {	 Run.motor_control(0,true);while(1);}
	 System::DelayMs(20);
	 }
	 */

	/*
	 to use pVarManager, you need to use Chrome to download the app by peter
	 link:
	 https://chrome.google.com/webstore/search/pgrapher?utm_source=chrome-ntp-icon
	 */

//-------------------------------------your code below----------------------------------------//
	System::Init();
	RunMode Kyle;
	//MUST initialize for using LCD and anything that contain function inside "System"
	//use tick
	//...
	bool IsPrint = false;
	bool IsProcess = false;
	float K = 15.0f;
	float T=0.47f;

	Button::Config btncfg;
	btncfg.is_active_low = true;
	btncfg.is_use_pull_resistor = false;
	btncfg.id = 0;
	btncfg.listener_trigger = Button::Config::Trigger::kDown;
	btncfg.listener = [&](const uint8_t)
	{
		IsPrint = !IsPrint;
		Kyle.switchLED(3);
		Kyle.beepbuzzer(100);
	};
	Button but0(btncfg);

	btncfg.id = 1;
	btncfg.listener_trigger = Button::Config::Trigger::kDown;
	btncfg.listener = [&](const uint8_t)
	{
		IsProcess = !IsProcess;
		Kyle.switchLED(2);
		Kyle.beepbuzzer(100);
	};
	Button but1(btncfg);

	Joystick::Config fwaycfg;
	fwaycfg.id = 0;

	fwaycfg.listener_triggers[static_cast<int>(Joystick::State::kUp)] =
			Joystick::Config::Trigger::kDown;
	fwaycfg.listeners[static_cast<int>(Joystick::State::kUp)] =
			[&](const uint8_t)
			{
				K+=1.0f;
				Kyle.beepbuzzer(100);
			};

	fwaycfg.listener_triggers[static_cast<int>(Joystick::State::kDown)] =
			Joystick::Config::Trigger::kDown;
	fwaycfg.listeners[static_cast<int>(Joystick::State::kDown)] =
			[&](const uint8_t)
			{
				K-=1.0f;
				Kyle.beepbuzzer(100);
			};

	fwaycfg.listener_triggers[static_cast<int>(Joystick::State::kLeft)] =
			Joystick::Config::Trigger::kDown;
	fwaycfg.listeners[static_cast<int>(Joystick::State::kLeft)] =
			[&](const uint8_t)
			{
				T-=0.01f;
				Kyle.beepbuzzer(100);
			};

	fwaycfg.listener_triggers[static_cast<int>(Joystick::State::kRight)] =
			Joystick::Config::Trigger::kDown;
	fwaycfg.listeners[static_cast<int>(Joystick::State::kRight)] =
			[&](const uint8_t)
			{
				T+=0.01f;
				Kyle.beepbuzzer(100);
			};

	Joystick joy(fwaycfg);


	Looper looper;
	ImageProcess imp;
	Planner pln;

//	Kyle.GetMotor().SetPower(150);
	Kyle.GetServo().SetDegree(900);
	Looper::Callback printraw =// configure the callback function for looper
			[&](const Timer::TimerInt request, const Timer::TimerInt)
			{
				Kyle.capture_image();
				Kyle.switchLED(1);
				if(IsProcess) {
					imp.FindEdge(Kyle.image,Kyle.edges,Kyle.bgstart,3,5);
					pln.Calc(Kyle.edges,Kyle.waypoints,Kyle.bgstart,Kyle.mid);
				}
				if(IsPrint) {
					Kyle.printRawCamGraph(0,0,Kyle.data);//print raw for better performance
					Kyle.printEdge(0,0);
					Kyle.printvalue(0,60,80,20,Kyle.mid,Lcd::kCyan);
					Kyle.printvalue(0,80,80,20,K*100,Lcd::kBlue);
					Kyle.printvalue(0,100,80,20,T*100,Lcd::kPurple);
					Kyle.printWaypoint(0,0);
					Kyle.GetLCD().SetRegion(Lcd::Rect(Kyle.mid,0,1,60));
					Kyle.GetLCD().FillColor(Lcd::kCyan);
				}
				Kyle.turningPID(Kyle.mid,K,T);
//				Kyle.motorPID(4000,K);
				looper.RunAfter(request, printraw);
			};
	Looper::Callback m_motorPID =// configure the callback function for looper
			[&](const Timer::TimerInt request, const Timer::TimerInt)
			{
				Kyle.GetMotor().SetPower(150);//TODO: adjust speed according to error from mid, i.e. the turning angle; add PID
				looper.RunAfter(request,m_motorPID);
			};

	Kyle.beepbuzzer(200);
	looper.RunAfter(20, printraw);
	looper.RunAfter(20, m_motorPID);
	looper.Loop();
	for (;;) {
	}
	looper.~Looper();
	Kyle.~RunMode();
	return 0;
}
