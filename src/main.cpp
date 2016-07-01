/*
 * main.cpp
 *
 * Author: Kyle
 * Adapted from code written by Peter
 * Copyright © 2015-2016 HKUST SmartCar Team. All rights reserved.
 * Refer to LICENSE for details
 */

#include <libbase/k60/watchdog.h>
#include <libbase/misc_types.h>
#include <libsc/button.h>
#include <libsc/joystick.h>
#include <libsc/lcd.h>
#include <libsc/st7735r.h>
#include <libsc/system.h>
#include <libsc/timer.h>
#include <libutil/looper.h>

#include "../inc/car.h"
#include "../inc/Planner.h"
#include "../inc/RunMode.h"
#include "../inc/ImageProcess.h"

#ifdef USE_PGRAPHER
#include <libutil/pGrapher.h>
#endif

using namespace libsc;

using namespace libbase::k60;

using namespace libutil;

int main(void) {
	Watchdog::Init();
	System::Init();

	RunMode Kyle;
	Looper looper;

	bool IsPrint = false;
	bool IsProcess = false;
	bool IsEditKp = false;
	int32_t dmid = 0;	//10*Kyle.mid, to look more significant on the graph
	bool stop = false;
	uint8_t stop_count=0;

	VarSet Selected;

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
				if(!Kyle.selecting_varset&&IsPrint) {
#ifdef ADJUST_CAM
			Kyle.m_brightness+=1;
			Kyle.GetCam().SetBrightness(Kyle.m_brightness);
#endif
			Selected.ideal_encoder_count+=50;
			Kyle.beepbuzzer(100);
		}
		else {
			Kyle.varset_index--;
#ifdef TESTSERVO
			Kyle.deg+=10;
#endif
			Kyle.beepbuzzer(100);
		}
	};

	fwaycfg.handlers[static_cast<int>(Joystick::State::kDown)] =
			[&](const uint8_t,const Joystick::State)
			{
				if(!Kyle.selecting_varset&&IsPrint) {
#ifdef ADJUST_CAM
			Kyle.m_brightness-=1;
			Kyle.GetCam().SetBrightness(Kyle.m_brightness);
#endif
			Selected.ideal_encoder_count-=50;
			Kyle.beepbuzzer(100);
		}
		else {
			Kyle.varset_index++;
#ifdef TESTSERVO
			Kyle.deg-=10;
#endif
			Kyle.beepbuzzer(100);
		}
	};

	fwaycfg.handlers[static_cast<int>(Joystick::State::kLeft)] =
			[&](const uint8_t,const Joystick::State)
			{
				if(!Kyle.selecting_varset&&IsPrint) {
#ifdef ADJUST_CAM
			Kyle.m_contrast-=1;
			Kyle.GetCam().SetContrast(Kyle.m_contrast);
#endif
			if(IsEditKp) Selected.r_Kp-=0.01f;
			else Selected.r_Kd-=0.5f;
			Kyle.beepbuzzer(100);
		}

	};

	fwaycfg.handlers[static_cast<int>(Joystick::State::kRight)] =
			[&](const uint8_t,const Joystick::State)
			{
				if(!Kyle.selecting_varset&&IsPrint) {
#ifdef ADJUST_CAM
			Kyle.m_contrast+=1;
			Kyle.GetCam().SetContrast(Kyle.m_contrast);
#endif
			if(IsEditKp) Selected.r_Kp+=0.01f;
			else Selected.r_Kd+=0.5f;
			Kyle.beepbuzzer(100);
		}

	};

	fwaycfg.handlers[static_cast<int>(Joystick::State::kSelect)] =
			[&](const uint8_t,const Joystick::State)
			{
				if(!Kyle.selecting_varset&&IsPrint) {
					IsEditKp=!IsEditKp;
					Kyle.switchLED(4,IsEditKp);
					Kyle.beepbuzzer(100);
				}
				else {
					Kyle.selecting_varset = false;
					Kyle.beepbuzzer(100);
				}

			};
	Joystick joy(fwaycfg);

	Kyle.beepbuzzer(200);
	Selected = Kyle.SelectVarSet();
	Planner pln;
	ImageProcess imp(Selected);
	Kyle.GetLCD().Clear();

#ifdef USE_PGRAPHER
	int16_t last_count = Selected.ideal_encoder_count;
	pGrapher mvar; //call constructor after selecting VarSet, in case memory addresses freak out

	/*-------configure tuning parameters below-----*/
//	mvar.addWatchedVar(&Kyle.real_encodercount, "Real Encoder");
#ifdef ADJUST_CAM
	mvar.addWatchedVar(&Kyle.m_brightness,"Brightness");
	mvar.addWatchedVar(&Kyle.m_contrast,"Contrast");
#endif
	mvar.addWatchedVar(&Kyle.encodercount, "Smoothed Encoder");
	mvar.addWatchedVar(&dmid, "Mid-line");
//	mvar.addSharedVar(&Selected.Kp, "Kp");
//	mvar.addSharedVar(&Selected.Ki, "Ki");
	mvar.addSharedVar(&Selected.l_Kp, "left Kp");
	mvar.addSharedVar(&Selected.l_Kd, "left Kd");
	mvar.addSharedVar(&Selected.r_Kp, "right Kp");
	mvar.addSharedVar(&Selected.r_Kd, "right Kd");
	mvar.addSharedVar(&Selected.KDec, "KDec");
	mvar.addSharedVar(&Selected.offset, "Offset");
	mvar.addSharedVar(&Selected.ideal_encoder_count, "Ideal Encoder");
	/*------configure tuning parameters above------*/
	pGrapher::OnReceiveListener mvarlistener =
			[&](const std::vector<Byte>& msg) {
				switch(msg[0]) {
					case 'w':
						Selected.ideal_encoder_count=last_count;
					break;
					case 's':
						last_count=Selected.ideal_encoder_count==0?last_count:Selected.ideal_encoder_count;
						Selected.ideal_encoder_count=0;
					break;
					case 'x':
						Selected.ideal_encoder_count=-last_count;
					break;
					case 'e':
						Selected.ideal_encoder_count+=50;
					break;
					case 'q':
						Selected.ideal_encoder_count-=50;
					break;
					case 'f':
					Kyle.beepbuzzer(100);
					System::DelayMs(50);
					Kyle.beepbuzzer(500);
					break;
				};
			};
	mvar.setOnReceiveListener(mvarlistener);
#endif

	Looper::Callback m_imp =	// configure the callback function for looper
			[&](const Timer::TimerInt request, const Timer::TimerInt)
			{
				Kyle.capture_image();
				imp.FindEdge(Kyle.data,Kyle.edges,Kyle.waypoints,Kyle.bgstart,5,Selected.offset,stop);
				pln.Calc(Kyle.waypoints,Kyle.bgstart,Kyle.mid);
#ifdef USE_LCD
			if(IsPrint) {
				Kyle.printRawCamGraph(1,0,Kyle.data);//print raw for better performance
				Kyle.printEdge(1,0);
				Kyle.printvalue(30,60,20,20,Kyle.mid,Lcd::kCyan);
				Kyle.printvalue(100,60,40,20,Selected.ideal_encoder_count,Lcd::kRed);
				Kyle.printvalue(25,80,55,20,Selected.l_Kp*100,Lcd::kBlue);
				Kyle.printvalue(25,100,55,20,Selected.l_Kd*100,Lcd::kPurple);
				Kyle.printWaypoint(0,0);
				Kyle.GetLCD().SetRegion(Lcd::Rect(Kyle.mid+1,0,1,60));
				Kyle.GetLCD().FillColor(Lcd::kCyan);
			}
#endif
			Kyle.switchLED(1);
//			if(stop)stop_count++;
//			else stop_count=0;
//			if(stop_count > 2)Selected.ideal_encoder_count = 0;
//			if(stop) Selected.ideal_encoder_count=0;
			dmid=10*Kyle.mid;//store in dmid for pGrapher
			if(IsProcess) Kyle.turningPID(Kyle.mid,Selected);
			Watchdog::Refresh();//LOL, feed or get bitten
		};
	Looper::Callback m_motorPID =// configure the callback function for looper
			[&](const Timer::TimerInt request, const Timer::TimerInt)
			{
				if(!IsPrint) Kyle.motorPID(Selected);	//when using LCD the system slows down dramatically, causing the motor to go crazy
#ifdef USE_PGRAPHER
			mvar.sendWatchData();
#endif
		};
	Kyle.switchLED(2, IsProcess);
	Kyle.switchLED(3, IsPrint);
	Kyle.printvalue(0, 60, 30, 20, "Mid=", Lcd::kCyan);
	Kyle.printvalue(60, 60, 30, 20, "PWR=", Lcd::kRed);
	Kyle.printvalue(0, 80, 25, 20, "Kp=", Lcd::kBlue);
	Kyle.printvalue(0, 100, 25, 20, "Kd=", Lcd::kPurple);
	looper.Repeat(20,m_imp,Looper::RepeatMode::kPrecise);
	looper.Repeat(20,m_motorPID,Looper::RepeatMode::kPrecise);
	looper.Loop();
	for (;;) {}
	return 0;
}
