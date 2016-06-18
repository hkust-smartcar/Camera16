/*
 * car.cpp
 *
 *  Created on: 28-3-2016
 *      Author: Kyle
 *      Adapted from code written by yungc
 *  Copyright © 2015-2016 HKUST SmartCar Team. All rights reserved.
 */

#include "../inc/car.h"

#include <libbase/misc_types.h>
#include <libsc/lcd.h>
#include <libsc/system.h>
#include <libsc/timer.h>
#include <cstdint>

#include "../inc/config.h"

using namespace std;

Car::Car() :
		bgstart(0), mid(39) {
	Led1 = new Led(GetLed1Config());
	Led2 = new Led(GetLed2Config());
	Led3 = new Led(GetLed3Config());
	Led4 = new Led(GetLed4Config());
#ifndef Use_Dir_Encoder
	encoder = new AbEncoder(GetAbEncoderConfig());
#else
	encoder = new DirEncoder(GetDirEncoderConfig());
#endif
	encoder->Update();
	servo = new TrsD05(GetServoConfig());
	servo->SetDegree(900);
	motor = new DirMotor(GetDirmotorConfig());
	LCD = new St7735r(GetLcdConfig());
	buzzer = new SimpleBuzzer(GetBuzzerConfig());
	batt = new BatteryMeter(GetBattConfig());
	cam = new k60::Ov7725(GetCamConfig());
	cam->Start();
	while (!cam->IsAvailable()) {
	};

	LcdTypewriter::Config LcdWconfig;
	LcdWconfig.lcd = LCD;
	LCDwriter = new LcdTypewriter(LcdWconfig);

	LcdConsole::Config LCDCConfig;
	LCDCConfig.lcd = LCD;
	LCDconsole = new LcdConsole(LCDCConfig);

	memset(this->data, 0, 600);
	//memset(this->image, false, 4800);
	memset(this->edges, 0, 120);
	memset(this->waypoints, 0, 60);
}
// for constructor, you can temporarily understand it as :
// initialize all variable ( & pointer)
// for the word "new", it is to create dynamic object.
// dynamic object means, it will always exist once you create it.
// opposite of dynamic is static.
//  int a; <----its static, meamory space of a will be release out the scope
//  int* a = new int; <----its dynamic, meamory space of datatype int will be preserved, and "new int" will
// return the address of that memory space. Thats why we use a pointer to store the address.

Car::~Car() {

	delete Led1;
	delete Led2;
	delete Led3;
	delete Led4;
	delete encoder;
	delete servo;
	delete motor;
	delete LCD;
	delete buzzer;
	delete LCDwriter;
	delete LCDconsole;
	cam->Stop();
	delete cam;

}
// for destructor, you can understand as following
// "how many times you "new" a object in constructor, same amount of "delete" you need to put in destructor"
// if you "new" a object but dont "delete" it, the object will forever exist.( for more search "memory leak" )

void Car::printvalue(int16_t value, int16_t color) {
	LCD->SetRegion(libsc::Lcd::Rect(0, 0, 128, 40));
	string Result;
	ostringstream convert;
	convert << value;
	Result = convert.str();
	const char *s = Result.c_str();
	LCDwriter->SetTextColor(color);
	LCDwriter->WriteString(s);
}

void Car::printvalue(int x, int y, int w, int h, int16_t value, int16_t color) {
	LCD->SetRegion(libsc::Lcd::Rect(x, y, w, h));
	std::string Result;
	std::ostringstream convert;
	convert << value;
	Result = convert.str();
	const char *s = Result.c_str();
	LCDwriter->SetTextColor(color);
	LCDwriter->WriteString(s);
}

void Car::printvalue(std::string Result) {
	LCD->SetRegion(libsc::Lcd::Rect(0, 0, 128, 40));
	const char *s = Result.c_str();
	LCDwriter->WriteString(s);
}

void Car::printvalue(int x, int y, int w, int h, std::string Result,
		int16_t color) {
	LCD->SetRegion(libsc::Lcd::Rect(x, y, w, h));
	const char *s = Result.c_str();
	LCDwriter->SetTextColor(color);
	LCDwriter->WriteString(s);
}

void Car::printRawCamGraph(const int8_t x, const int8_t y, Byte* data) {
	LCD->SetRegion(Lcd::Rect(x, y, 80, 60));
	LCD->FillBits(0, 0xFFFF, data, 80 * 60);
}

void Car::printEdge(const int8_t xpos, const int8_t ypos) {

	/*-----print edges and if at edge or not-----*/
	for (int8_t row = bgstart + 1; row < 60; row++) {
		for (int8_t ind = 0; ind < 60; ind++) {
			LCD->SetRegion(
					Lcd::Rect(xpos + edges[GetL(ind)], ypos + ind, 1, 1)); //print left edge, AKA first half of edge[120]
			LCD->FillColor(Lcd::kRed);
			LCD->SetRegion(
					Lcd::Rect(xpos + edges[GetR(ind)], ypos + ind, 1, 1)); //print right edge, AKA second half of edge[120]
			LCD->FillColor(Lcd::kBlue);
		}
	}

	/*-----print bgstart-----*/
	LCD->SetRegion(Lcd::Rect(xpos, ypos + bgstart, 80, 1));	//draw where bg start
	LCD->FillColor(Lcd::kYellow);

}

void Car::printWaypoint(const int8_t xpos, const int8_t ypos) {
	for (int8_t row = bgstart + 1; row < 60; row++) {
		LCD->SetRegion(Lcd::Rect(xpos + waypoints[row], ypos + row, 1, 1));
		LCD->FillColor(Lcd::kPurple);
	}
}

void Car::printline(int16_t value, uint16_t color) {
	LCD->SetRegion(libsc::Lcd::Rect(0, value * 160 / 255, 128, 1));
	LCD->FillColor(color);
}

void Car::clearLcd(uint16_t color) {
	LCD->Clear(color);
}

void Car::blinkLED(int8_t id, int delay_time, int persist_time) {
	libsc::Led* LedToBlink;
	switch (id) {
	case 1:
		LedToBlink = this->Led1;
		break;
	case 2:
		LedToBlink = this->Led2;
		break;
	case 3:
		LedToBlink = this->Led3;
		break;
	case 4:
		LedToBlink = this->Led4;
		break;
	}
	LedToBlink->SetEnable(true);
	while ((persist_time - delay_time) >= 0) {
		LedToBlink->Switch();
		libsc::System::DelayMs(delay_time);
		persist_time -= delay_time;
	}
	LedToBlink->SetEnable(false);

}

void Car::beepbuzzer(uint32_t t) {
	Timer::TimerInt m_t = System::Time();
	this->buzzer->SetBeep(true);
	while (System::Time() < m_t + t) {
	}
	this->buzzer->SetBeep(false);
}

void Car::switchLED(int8_t id) {
	libsc::Led* LedToBlink;
	switch (id) {
	case 1:
		LedToBlink = this->Led1;
		break;
	case 2:
		LedToBlink = this->Led2;
		break;
	case 3:
		LedToBlink = this->Led3;
		break;
	case 4:
		LedToBlink = this->Led4;
		break;
	default:
		return;
		break;
	}
	LedToBlink->Switch();
}

void Car::switchLED(int8_t id, bool isEnable) {
	libsc::Led* LedToBlink;
	switch (id) {
	case 1:
		LedToBlink = this->Led1;
		break;
	case 2:
		LedToBlink = this->Led2;
		break;
	case 3:
		LedToBlink = this->Led3;
		break;
	case 4:
		LedToBlink = this->Led4;
		break;
	default:
		return;
		break;
	}
	LedToBlink->SetEnable(isEnable);
}

void Car::capture_image(void) {

	// capture raw image
	memcpy(this->data, this->cam->LockBuffer(), 600);
	this->cam->UnlockBuffer();

}
