/*
 * car.cpp
 *
 *  Created on: 28-3-2016
 *      Author: Kyle
 *      Adapted from code written by yungc
 *  Copyright © 2015-2016 HKUST SmartCar Team. All rights reserved.
 */
#include "car.h"
#include "config.h"
#define data_size 600

using namespace std;

Car::Car() {
	Led1 = new Led(GetLed1Config());
	Led2 = new Led(GetLed2Config());
	Led3 = new Led(GetLed3Config());
	Led4 = new Led(GetLed4Config());
	encoder = new AbEncoder(GetAbEncoderConfig());
	servo = new TrsD05(GetServoConfig());
	motor = new DirMotor(GetDirmotorConfig());
	LCD = new St7735r(GetLcdConfig());
	buzzer = new SimpleBuzzer(GetBuzzerConfig());
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

	memset(data, 0, 600);
	memset(image, false, true * 80 * 60);
	memset(edges, 0, 120);
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

void Car::printvalue(int x, int y, int w, int h, std::string Result) {
	LCD->SetRegion(libsc::Lcd::Rect(x, y, w, h));
	const char *s = Result.c_str();
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

void Car::printWaypoint(int8_t xpos, int8_t ypos) {
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
		LedToBlink = Led1;
		break;
	case 2:
		LedToBlink = Led2;
		break;
	case 3:
		LedToBlink = Led3;
		break;
	case 4:
		LedToBlink = Led4;
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
	buzzer->SetBeep(true);
	System::DelayMs(t);
	buzzer->SetBeep(false);
}

void Car::switchLED(int8_t id) {
	libsc::Led* LedToBlink;
	switch (id) {
	case 1:
		LedToBlink = Led1;
		break;
	case 2:
		LedToBlink = Led2;
		break;
	case 3:
		LedToBlink = Led3;
		break;
	case 4:
		LedToBlink = Led4;
		break;
	}
	LedToBlink->Switch();
}

bool Car::GetPixel(const Byte* src, const int8_t x, const int8_t y) {
	//	const int offset = x/8 + (y * image_width / 8);
	const int offset = x / 8 + (y * 80 / 8);

	//	return (src[offset] << (x%8) & 0x80) ? 0 : 1;
	return (src[offset] << (x % 8) & 0x80) ? false : true;
}

void Car::capture_image(void) {

	// capture raw image
	memcpy(data, cam->LockBuffer(), data_size);
	cam->UnlockBuffer();

	// divide image
	for (int8_t col = 0; col < 80; col++) {
		for (int8_t row = 0; row < 60; row++) {
			image[col][row] = GetPixel(data, col, row);
		}
	}
}
