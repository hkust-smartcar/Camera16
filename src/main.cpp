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
	Watchdog::Init();
	System::Init();

	RunMode Kyle;
	Looper looper;
	ImageProcess imp;
	Planner pln;
	//MUST initialize for using LCD and anything that contain function inside "System"
	//use tick
	//...
	bool IsPrint = false;
	bool IsProcess = false;
	bool IsEditKd = false;
	int16_t real_encoder_count = 0;
	uint32_t dmid = 0;	//10*Kyle.mid, to look more significant on the graph
	bool stop = false;

	/*-----variables waiting to be assigned-----*/
	int16_t ideal_encoder_count;
	float K;
	float T;
	float Kp;
	float Ki;
	float Kd;
	int8_t offset;
	int8_t plnstart;

	Button::Config btncfg;
	btncfg.is_active_low = true;
	btncfg.is_use_pull_resistor = false;
	btncfg.listener_trigger = Button::Config::Trigger::kDown;

	btncfg.id = 0;
	btncfg.listener = [&](const uint8_t)
	{
		if(!Kyle.selecting_varset) {//when finished selecting varset, trigger the following
				IsPrint = !IsPrint;
				Kyle.switchLED(3,IsPrint);
				Kyle.beepbuzzer(100);
			}
		};
	Button btn0(btncfg);

	btncfg.id = 1;
	btncfg.listener = [&](const uint8_t)
	{
		if(!Kyle.selecting_varset) {
			IsProcess = !IsProcess;
			Kyle.switchLED(2,IsProcess);
			Kyle.beepbuzzer(100);
		}
	};
	Button btn1(btncfg);

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
	fwaycfg.listener_triggers[static_cast<int>(Joystick::State::kSelect)] =
			Joystick::Config::Trigger::kDown;

	fwaycfg.handlers[static_cast<int>(Joystick::State::kUp)] =
			[&](const uint8_t,const Joystick::State)
			{
				if(!Kyle.selecting_varset) {
					ideal_encoder_count+=50;
					Kyle.beepbuzzer(100);
				}
				else {
					Kyle.varset_index--;
					//Kyle.deg+=10;
					Kyle.beepbuzzer(100);
				}
			};

	fwaycfg.handlers[static_cast<int>(Joystick::State::kDown)] =
			[&](const uint8_t,const Joystick::State)
			{
				if(!Kyle.selecting_varset) {
					ideal_encoder_count-=50;
					Kyle.beepbuzzer(100);
				}
				else {
					Kyle.varset_index++;
					//Kyle.deg-=10;
					Kyle.beepbuzzer(100);
				}
			};

	fwaycfg.handlers[static_cast<int>(Joystick::State::kLeft)] =
			[&](const uint8_t,const Joystick::State)
			{
				if(!Kyle.selecting_varset) {
					if(IsEditKd) T-=0.01f;
					else K-=0.1f;
					Kyle.beepbuzzer(100);
				}

			};

	fwaycfg.handlers[static_cast<int>(Joystick::State::kRight)] =
			[&](const uint8_t,const Joystick::State)
			{
				if(!Kyle.selecting_varset) {
					if(IsEditKd) T+=0.01f;
					else K+=0.1f;
					Kyle.beepbuzzer(100);
				}

			};

	fwaycfg.handlers[static_cast<int>(Joystick::State::kSelect)] =
			[&](const uint8_t,const Joystick::State)
			{
				if(!Kyle.selecting_varset) {
					IsEditKd=!IsEditKd;
					Kyle.switchLED(4,IsEditKd);
					Kyle.beepbuzzer(100);
				}
				else {
					Kyle.selecting_varset = false;
					Kyle.beepbuzzer(100);
				}

			};
	Joystick joy(fwaycfg);

	Kyle.beepbuzzer(200);
	VarSet Selected = Kyle.SelectVarSet();
	Kyle.GetLCD().Clear();

	/*------assign VarSet variables-----*/
	ideal_encoder_count = Selected.ideal_encoder_count;
	K = Selected.K;
	T = Selected.T;
	Kp = Selected.Kp;
	Ki = Selected.Ki;
	Kd = Selected.Kd;
	offset = Selected.offset;
	plnstart = Selected.plnstart;

	pVarManager mvar; //call constructor after selecting VarSet, in case memory addresses freak out

	/*-------configure tuning parameters below-----*/
	mvar.addWatchedVar(&real_encoder_count, "Real Encoder");
	mvar.addWatchedVar(&Kyle.ideal_motor_speed, "Ideal Motor");
	mvar.addWatchedVar(&dmid, "Mid-line");
	mvar.addSharedVar(&Kp, "Kp");
	mvar.addSharedVar(&Ki, "Ki");
	mvar.addSharedVar(&Kd, "Kd");
	mvar.addSharedVar(&T, "servoK");
	mvar.addSharedVar(&K, "servoKd");
	mvar.addSharedVar(&offset, "Offset");
	mvar.addSharedVar(&plnstart, "PLNStart");
	mvar.addSharedVar(&ideal_encoder_count, "Ideal Encoder");
	/*------configure tuning parameters above------*/

	Looper::Callback m_imp =	// configure the callback function for looper
			[&](const Timer::TimerInt request, const Timer::TimerInt)
			{
				Kyle.capture_image();
				Kyle.switchLED(1);
				if(IsPrint) {
					Kyle.printRawCamGraph(1,0,Kyle.data);//print raw for better performance
					Kyle.printEdge(1,0);
					Kyle.printvalue(0,60,30,20,"Mid=",Lcd::kCyan);
					Kyle.printvalue(30,60,20,20,Kyle.mid,Lcd::kCyan);
					Kyle.printvalue(60,60,30,20,"PWR=",Lcd::kRed);
					Kyle.printvalue(100,60,40,20,ideal_encoder_count,Lcd::kRed);
					Kyle.printvalue(0,80,25,20,"Kp=",Lcd::kBlue);
					Kyle.printvalue(25,80,55,20,T*100,Lcd::kBlue);
					Kyle.printvalue(0,100,25,20,"Kd=",Lcd::kPurple);
					Kyle.printvalue(25,100,55,20,K*100,Lcd::kPurple);
					Kyle.printWaypoint(0,0);
					Kyle.GetLCD().SetRegion(Lcd::Rect(Kyle.mid+1,0,1,60));
					Kyle.GetLCD().FillColor(Lcd::kCyan);
					Kyle.GetLCD().SetRegion(Lcd::Rect(0,125,80,25));
					Kyle.GetLCD().FillColor(ideal_encoder_count?Lcd::kGreen:Lcd::kRed);
				}
				imp.FindEdge(Kyle.image,Kyle.edges,Kyle.bgstart,2,offset,stop);
				if(stop) {
					ideal_encoder_count = 0;
				}
				pln.Calc(Kyle.edges,Kyle.waypoints,Kyle.bgstart,Kyle.mid,plnstart);
				dmid=10*Kyle.mid;	//store in dmid for pGrapher
				if(IsProcess) Kyle.turningPID(Kyle.mid,K,T);
				Watchdog::Refresh();//LOL, feed or get bitten
				looper.RunAfter(request, m_imp);
			};
	Looper::Callback m_motorPID =// configure the callback function for looper
			[&](const Timer::TimerInt request, const Timer::TimerInt)
			{
				if(!IsPrint) Kyle.motorPID(ideal_encoder_count,Kp,Ki,Kd);//when using LCD the system slows down dramatically, causing the motor to go crazy
				real_encoder_count=-Kyle.GetEnc().GetCount();
				mvar.sendWatchData();
				looper.RunAfter(request,m_motorPID);
			};

	Kyle.switchLED(2, IsProcess);
	Kyle.switchLED(3, IsPrint);
	looper.RunAfter(20, m_imp);
	looper.RunAfter(20, m_motorPID);
	looper.Loop();
	for (;;) {
	}
	return 0;
}
