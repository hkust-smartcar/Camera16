/*
 * main.cpp
 *
 * Author: Kyle
 * Adapted from code written by Peter
 * Copyright © 2015-2016 HKUST SmartCar Team. All rights reserved.
 * Refer to LICENSE for details
 */

#include <libbase/k60/watchdog.h>
#include <libsc/button.h>
#include <libsc/encoder.h>
#include <libsc/joystick.h>
#include <libsc/lcd.h>
#include <libsc/st7735r.h>
#include <libsc/system.h>
#include <libsc/timer.h>
#include <libutil/looper.h>
#include <stdint.h>
#include <functional>

#include "../inc/car.h"
#include "../inc/ImageProcess.h"
#include "../inc/pVarManager.h"
#include "../inc/Planner.h"
#include "../inc/RunMode.h"

using namespace libsc;

using namespace libbase::k60;

using namespace libutil;

int main(void) {

//code for plotting graph for a equation of y = mx +c, where y and x are encoder counting or motor PWM
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
	Watchdog::Init();
	System::Init();

	RunMode Kyle;
	Looper looper;
	ImageProcess imp;
	Planner pln;
	pVarManager mvar;
	//MUST initialize for using LCD and anything that contain function inside "System"
	//use tick
	//...
	bool IsPrint = false;
	bool IsProcess = false;
	bool IsEditKd = false;
	float K = 5.0f;
	float T = 0.45f;
	int16_t ideal_encoder_count = 0;
	int16_t real_encoder_count = 0;
	uint32_t dmid = 0;	//10*Kyle.mid, to look more significant on the graph
	float Kp = 0.37f;
	float Ki = 0.02f;
	float Kd = 0.15f;
	int8_t offset=5;

	Button::Config btncfg;
	btncfg.is_active_low = true;
	btncfg.is_use_pull_resistor = false;
	btncfg.listener_trigger = Button::Config::Trigger::kDown;

	btncfg.id = 0;
	btncfg.listener = [&](const uint8_t)
	{
		IsPrint = !IsPrint;
		Kyle.switchLED(3);
		Kyle.beepbuzzer(100);
	};
	Button but0(btncfg);

	btncfg.id = 1;
	btncfg.listener = [&](const uint8_t)
	{
		IsProcess = !IsProcess;
		Kyle.switchLED(2);
		Kyle.beepbuzzer(100);
	};
	Button but1(btncfg);

	Joystick::Config fwaycfg;
	fwaycfg.is_active_low = true;
	fwaycfg.id = 0;

	fwaycfg.listener_triggers[static_cast<int>(Joystick::State::kUp)] =
			Joystick::Config::Trigger::kDown;
	fwaycfg.listener_triggers[static_cast<int>(Joystick::State::kDown)] =
			Joystick::Config::Trigger::kDown;
	fwaycfg.listener_triggers[static_cast<int>(Joystick::State::kLeft)] =
			Joystick::Config::Trigger::kDown;
	fwaycfg.listener_triggers[static_cast<int>(Joystick::State::kRight)] =
			Joystick::Config::Trigger::kDown;

	fwaycfg.handlers[static_cast<int>(Joystick::State::kUp)] =
			[&](const uint8_t,const Joystick::State)
			{
				ideal_encoder_count+=50;
				Kyle.beepbuzzer(100);
			};

	fwaycfg.handlers[static_cast<int>(Joystick::State::kDown)] =
			[&](const uint8_t,const Joystick::State)
			{
				ideal_encoder_count-=50;
				Kyle.beepbuzzer(100);
			};

	fwaycfg.listener_triggers[static_cast<int>(Joystick::State::kLeft)] =
			Joystick::Config::Trigger::kDown;
	fwaycfg.handlers[static_cast<int>(Joystick::State::kLeft)] =
			[&](const uint8_t,const Joystick::State)
			{
				if(IsEditKd) T-=0.01f;
				else K-=0.5f;
				Kyle.beepbuzzer(100);
			};

	fwaycfg.listener_triggers[static_cast<int>(Joystick::State::kRight)] =
			Joystick::Config::Trigger::kDown;
	fwaycfg.handlers[static_cast<int>(Joystick::State::kRight)] =
			[&](const uint8_t,const Joystick::State)
			{
				if(IsEditKd) T+=0.01f;
				else K+=0.5;
				Kyle.beepbuzzer(100);
			};

	fwaycfg.listener_triggers[static_cast<int>(Joystick::State::kSelect)] =
			Joystick::Config::Trigger::kDown;
	fwaycfg.handlers[static_cast<int>(Joystick::State::kSelect)] =
			[&](const uint8_t,const Joystick::State)
			{
				IsEditKd=!IsEditKd;
				Kyle.switchLED(4);
				Kyle.beepbuzzer(100);
			};
	Joystick joy(fwaycfg);

	/*-------configure tuning parameters below-----*/
	mvar.addWatchedVar(&real_encoder_count, "Real Encoder");
	mvar.addWatchedVar(&Kyle.ideal_motor_speed, "Ideal Motor");
	mvar.addWatchedVar(&dmid, "Mid-line");
	mvar.addSharedVar(&Kp, "Kp");
	mvar.addSharedVar(&Ki, "Ki");
	mvar.addSharedVar(&Kd, "Kd");
	mvar.addSharedVar(&K, "servoK");
	mvar.addSharedVar(&T, "servoKd");
	mvar.addSharedVar(&offset, "Offset");
	mvar.addSharedVar(&ideal_encoder_count, "Ideal Encoder");
	/*------configure tuning parameters above------*/

	Looper::Callback m_imp =	// configure the callback function for looper
			[&](const Timer::TimerInt request, const Timer::TimerInt)
			{
				Kyle.capture_image();
				Kyle.switchLED(1);
				if(IsPrint) {
					Kyle.printRawCamGraph(0,0,Kyle.data);//print raw for better performance
					Kyle.printEdge(0,0);
					Kyle.printvalue(0,60,80,20,Kyle.mid,Lcd::kCyan);
					Kyle.printvalue(0,80,80,20,K,Lcd::kBlue);
					Kyle.printvalue(0,100,80,20,T*100,Lcd::kPurple);
					Kyle.printWaypoint(0,0);
					Kyle.GetLCD().SetRegion(Lcd::Rect(Kyle.mid,0,1,60));
					Kyle.GetLCD().FillColor(Lcd::kCyan);
					if(Kyle.stop)Kyle.printvalue(0,120,80,20,"end la");
					else Kyle.printvalue(0,120,80,20,"continue");

				}
				imp.FindEdge(Kyle.image,Kyle.edges,Kyle.bgstart,2,offset,Kyle.stop);
				pln.Calc(Kyle.edges,Kyle.waypoints,Kyle.bgstart,Kyle.mid);
				if(IsProcess) Kyle.turningPID(Kyle.mid,K,T);
				Watchdog::Refresh();	//LOL, feed or get bitten
				looper.RunAfter(request, m_imp);
			};
	Looper::Callback m_motorPID =// configure the callback function for looper
			[&](const Timer::TimerInt request, const Timer::TimerInt)
			{
				if(!IsPrint) Kyle.motorPID(ideal_encoder_count,Kp,Ki,Kd);//when using LCD the system slows down dramatically, causing the motor to go crazy
				real_encoder_count=-Kyle.GetEnc().GetCount();
				dmid=10*Kyle.mid;//store in dmid for pGrapher
				mvar.sendWatchData();
				looper.RunAfter(request,m_motorPID);
			};

	Kyle.beepbuzzer(200);
	Kyle.switchLED(2);
	Kyle.switchLED(3);
	looper.RunAfter(20, m_imp);
	looper.RunAfter(20, m_motorPID);
	looper.Loop();
	while (true);
}
