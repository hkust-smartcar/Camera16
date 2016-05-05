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
#include "pVarManager.h"
#include "RunMode.h"
#include <ImageProcess.h>
#include <Planner.h>
#include <libutil/looper.h>
#include <libbase/k60/watchdog.h>

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
	float K = 60.0f;
	float T=0.47f;
	int16_t ideal_encoder_count=0;
	int16_t real_encoder_count=0;
	uint32_t dmid=0;//10*Kyle.mid, to look more significant on the graph
	float Kp=0.37f;
	float Ki=0.01f;
	float Kd=0.1f;

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
				ideal_encoder_count+=100;
				Kyle.beepbuzzer(100);
			};

	fwaycfg.listener_triggers[static_cast<int>(Joystick::State::kDown)] =
			Joystick::Config::Trigger::kDown;
	fwaycfg.listeners[static_cast<int>(Joystick::State::kDown)] =
			[&](const uint8_t)
			{
				ideal_encoder_count-=100;
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

	fwaycfg.listener_triggers[static_cast<int>(Joystick::State::kSelect)] =
			Joystick::Config::Trigger::kDown;
	fwaycfg.listeners[static_cast<int>(Joystick::State::kSelect)] =
			[&](const uint8_t)//shutdown process, to prevent potential memory leak
			{
				Kyle.beepbuzzer(100);
				Kyle.switchLED(3);
				Timer::TimerInt m_t=System::Time();
				Timer::TimerInt shut_down_delay=3000;
				while(System::Time()<=m_t+shut_down_delay){
					if(but0.IsDown()||but1.IsDown()){
						looper.~Looper();
						Kyle.~RunMode();
						mvar.~pVarManager();
						for(;;){
							Watchdog::GoodDoggie();//treat your watchdog well
							System::DelayMs(500);
						}
					}
					if((System::Time()-m_t)%500==0){
						Kyle.beepbuzzer(100);
					}
					Watchdog::Pet();
					System::DelayMs(50);
				}
				Kyle.switchLED(3);
			};

	Joystick joy(fwaycfg);

	/*-------configure tuning parameters below-----*/
	mvar.addWatchedVar(&real_encoder_count,"Real Encoder");
	mvar.addWatchedVar(&Kyle.ideal_motor_speed,"Ideal Motor");
	mvar.addWatchedVar(&dmid,"Mid-line");
	mvar.addSharedVar(&Kp,"Kp");
	mvar.addSharedVar(&Ki,"Ki");
	mvar.addSharedVar(&Kd,"Kd");
	mvar.addSharedVar(&K,"servoK");
	mvar.addSharedVar(&T,"servoKd");
	mvar.addSharedVar(&ideal_encoder_count,"Ideal Encoder");
	/*------configure tuning parameters above------*/

	Looper::Callback m_imp =// configure the callback function for looper
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
					Kyle.printvalue(0,80,80,20,real_encoder_count,Lcd::kBlue);
					Kyle.printvalue(0,100,80,20,T*100,Lcd::kPurple);
					Kyle.printWaypoint(0,0);
					Kyle.GetLCD().SetRegion(Lcd::Rect(Kyle.mid,0,1,60));
					Kyle.GetLCD().FillColor(Lcd::kCyan);
				}
				Kyle.turningPID(Kyle.mid,K,T);
//				Kyle.motorPID(4000,K);
				Watchdog::GoodDoggie();//LOL, feed or get bitten
				looper.RunAfter(request, m_imp);
			};
	Looper::Callback m_motorPID =// configure the callback function for looper
			[&](const Timer::TimerInt request, const Timer::TimerInt)
			{
//				Kyle.GetMotor().SetPower(150);//TODO: adjust speed according to error from mid, i.e. the turning angle; add PID
				if(!IsPrint) Kyle.motorPID(ideal_encoder_count,Kp,Ki,Kd);//when using LCD the system slows down dramatically, causing the motor to go crazy
				real_encoder_count=-Kyle.GetEnc().GetCount();
				dmid=10*Kyle.mid;
				mvar.sendWatchData();
				looper.RunAfter(request,m_motorPID);
			};

	Kyle.beepbuzzer(200);
	looper.RunAfter(20, m_imp);
	looper.RunAfter(20, m_motorPID);
	looper.Loop();
	for (;;) {}
	return 0;
}
