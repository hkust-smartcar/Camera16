/*
 * main.cpp
 *
 * Author: Kyle, Judy, Bling Bling
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
	uint32_t start_time=0;
	bool stop = false;

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
			return;
		};
	Button btn0(btncfg);

	btncfg.id = 1;
	btncfg.listener = [&](const uint8_t)
	{
		if(!Kyle.selecting_varset) {
			Kyle.beepbuzzer(100);
			start_time=System::Time();
#ifndef USE_PGRAPHER
			uint32_t t0=System::Time();
			uint32_t t=t0;
			int8_t counter=0;
			for(;;) {
				if(t!=System::Time()) {
					t=System::Time();
					if((t-t0)%500==0) {
						Watchdog::GoodDoggie();
						counter++;
						Kyle.beepbuzzer(100);
					}
					if(counter>=4) break;
				}
			}
#endif
			IsProcess = !IsProcess;
			Kyle.switchLED(2,IsProcess);
		}
	};
	Button btn1(btncfg);

	btncfg.id = 2;
	btncfg.listener = NULL;
	Button btn2(btncfg);

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
#ifdef ADJUST_CAM
			Kyle.m_brightness+=1;
			Kyle.GetCam().SetBrightness(Kyle.m_brightness);
#endif
			if(btn2.IsDown()) Selected.KDec-=0.1f;
			else Selected.ideal_encoder_count-=50;
			Kyle.beepbuzzer(100);
		}
		else {
			Kyle.varset_index--;
#ifdef TESTSERVO
			Kyle.deg+=5;
#endif
			Kyle.beepbuzzer(100);
		}
	};

	fwaycfg.handlers[static_cast<int>(Joystick::State::kDown)] =
			[&](const uint8_t,const Joystick::State)
			{
				if(!Kyle.selecting_varset) {
#ifdef ADJUST_CAM
			Kyle.m_brightness-=1;
			Kyle.GetCam().SetBrightness(Kyle.m_brightness);
#endif
			if(btn2.IsDown()) Selected.KDec+=0.1f;
			else Selected.ideal_encoder_count+=50;
			Kyle.beepbuzzer(100);
		}
		else {
			Kyle.varset_index++;
#ifdef TESTSERVO
			Kyle.deg-=5;
#endif
			Kyle.beepbuzzer(100);
		}
	};

	fwaycfg.handlers[static_cast<int>(Joystick::State::kRight)] =
			[&](const uint8_t,const Joystick::State)
			{
				if(!Kyle.selecting_varset) {
#ifdef ADJUST_CAM
			Kyle.m_contrast-=1;
			Kyle.GetCam().SetContrast(Kyle.m_contrast);
#endif
			if(IsEditKp) {
				if(btn2.IsDown()) Selected.starting_row-=1;
				else {
					Selected.r_Kp-=0.01f;
					Selected.l_Kp-=0.01f;
				}
			}
			else {
				if(btn2.IsDown())Selected.starting_row-=1;
				else {
					Selected.r_Kd-=1;
					Selected.l_Kd-=1;
				}
			}
			Kyle.beepbuzzer(100);
		}
	};

	fwaycfg.handlers[static_cast<int>(Joystick::State::kLeft)] =
			[&](const uint8_t,const Joystick::State)
			{
				if(!Kyle.selecting_varset) {
#ifdef ADJUST_CAM
			Kyle.m_contrast+=1;
			Kyle.GetCam().SetContrast(Kyle.m_contrast);
#endif
			if(IsEditKp) {
				if(btn2.IsDown()) Selected.starting_row+=1;
				else {
					Selected.r_Kp+=0.01f;
					Selected.l_Kp+=0.01f;
				}
			}
			else {
				if(btn2.IsDown()) Selected.starting_row+=1;
				else {
					Selected.r_Kd+=1;
					Selected.l_Kd+=1;
				}
			}
			Kyle.beepbuzzer(100);
		}
	};

	fwaycfg.handlers[static_cast<int>(Joystick::State::kSelect)] =
			[&](const uint8_t,const Joystick::State)
			{
#ifndef TESTSERVO
			if(!Kyle.selecting_varset) {
				IsEditKp=!IsEditKp;
				Kyle.switchLED(4,IsEditKp);
				Kyle.beepbuzzer(100);
			}
			else {
				Kyle.selecting_varset = false;
				Kyle.beepbuzzer(100);
			}
#endif

		};
	Joystick joy(fwaycfg);

	Kyle.beepbuzzer(200);
	Selected = Kyle.SelectVarSet();
	Planner pln(Selected.starting_row);
	int8_t prev_starting_row = Selected.starting_row;
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
	mvar.addWatchedVar(&Kyle.ideal_servo_degree, "deg");
	mvar.addSharedVar(&Selected.Kp, "Kp");
//	mvar.addSharedVar(&Selected.Ki, "Ki");
	mvar.addSharedVar(&Selected.l_Kp, "left Kp");
	mvar.addSharedVar(&Selected.l_Kd, "left Kd");
	mvar.addSharedVar(&Selected.r_Kp, "right Kp");
	mvar.addSharedVar(&Selected.r_Kd, "right Kd");
	mvar.addSharedVar(&Selected.KDec, "KDec");
	mvar.addSharedVar(&Selected.starting_row, "Start Row");
//	mvar.addSharedVar(&Selected.offset, "Offset");
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

	Looper::Callback m_loop =	// configure the callback function for looper
			[&](const Timer::TimerInt request, const Timer::TimerInt)
			{
				Kyle.capture_image();
				imp.FindEdge(Kyle.data,Kyle.edges,Kyle.waypoints,Kyle.bgstart,5,Selected.offset,stop,Kyle.IsCross);
				if(Selected.starting_row!=prev_starting_row) {
					prev_starting_row=Selected.starting_row;
					pln.ChangeWeight(Selected.starting_row);
				}
				pln.Calc(Kyle.waypoints,Kyle.bgstart,Kyle.mid);
#ifdef USE_LCD
			if(IsPrint) {
				Kyle.printRawCamGraph(1,0,Kyle.data);//print raw for better performance
				Kyle.printEdge(1,0);
				Kyle.printvalue(30,60,20,20,Kyle.mid,Lcd::kCyan);
				Kyle.printvalue(100,60,40,20,Selected.ideal_encoder_count,Lcd::kRed);
				Kyle.printvalue(40,80,30,20,Selected.l_Kp*100,Lcd::kBlue);
				Kyle.printvalue(40,100,55,20,Selected.l_Kd,Lcd::kPurple);
				Kyle.printvalue(40,120,30,20,Selected.r_Kp*100,Lcd::kGreen);
				Kyle.printvalue(40,140,55,20,Selected.r_Kd,Lcd::kWhite);
				Kyle.printvalue(100,100,28,20,Selected.KDec*10, Lcd::kYellow);
				Kyle.printvalue(100,140,28,20,Selected.starting_row, Lcd::kYellow);
				Kyle.printWaypoint(0,0);
				Kyle.GetLCD().SetRegion(Lcd::Rect(Kyle.mid+1,0,1,60));
				Kyle.GetLCD().FillColor(Lcd::kCyan);
			}
#endif
			Kyle.switchLED(1);
			if(Selected.allow_stop&&stop&&System::Time()-start_time>=5000) {
				Kyle.GetMotor().SetPower(0);
				for(;;){
					Kyle.capture_image();
					imp.FindEdge(Kyle.data,Kyle.edges,Kyle.waypoints,Kyle.bgstart,5,Selected.offset,stop,Kyle.IsCross);
					pln.Calc(Kyle.waypoints,Kyle.bgstart,Kyle.mid);
					Kyle.turningPID(Kyle.mid,Selected,Kyle.IsCross);
					Watchdog::GoodDoggie();
					System::DelayMs(20);
				}
			}
			dmid=10*Kyle.mid;	//store in dmid for pGrapher
			if(!IsPrint&&IsProcess) Kyle.motorPID(Selected);//when using LCD the system slows down dramatically, causing the motor to go crazy
#ifdef USE_PGRAPHER
			mvar.sendWatchData();
#endif
			if(IsProcess) Kyle.turningPID(Kyle.mid,Selected,Kyle.IsCross);
			Watchdog::Refresh();	//LOL, feed or get bitten
		};

	Kyle.switchLED(2, IsProcess);
	Kyle.switchLED(3, IsPrint);
	Kyle.printvalue(0, 60, 30, 20, "Mid=", Lcd::kCyan);
	Kyle.printvalue(60, 60, 30, 20, "PWR=", Lcd::kRed);
	Kyle.printvalue(0, 80, 30, 20, "LKp=", Lcd::kBlue);
	Kyle.printvalue(0, 100, 30, 20, "LKd=", Lcd::kPurple);
	Kyle.printvalue(0, 120, 30, 20, "RKp=", Lcd::kGreen);
	Kyle.printvalue(0, 140, 30, 20, "RKd=", Lcd::kWhite);
	Kyle.printvalue(70, 80, 50, 20, "KDec=", Lcd::kYellow);
	Kyle.printvalue(70, 120, 50, 20, "SRow=", Lcd::kYellow);
	looper.Repeat(20, m_loop, Looper::RepeatMode::kLoose);
	looper.Loop();
	for (;;) {
	}
	return 0;
}
