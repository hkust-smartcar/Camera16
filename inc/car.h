/*
 * car.h
 *
 *  Created on: 28-3-2016
 *      Author: Kyle
 *      Adapted from code written by yungc
 *  Copyright © 2015-2016 HKUST SmartCar Team. All rights reserved.
 *
 * Control hardwares, and store image-processing-related data to bypass annoying pointers.
 */

#pragma once
//#define Use_Dir_Encoder

#include <array>
#include <libsc/led.h>
#include <libbase/k60/mcg.h>
#ifndef Use_Dir_Encoder
#include <libsc/ab_encoder.h>
#else
#include <libsc/dir_encoder.h>
#endif
#include <libsc/trs_d05.h>
#include <libsc/k60/ov7725.h>
#include <libsc/button.h>
#include <libsc/joystick.h>
#include <libsc/st7735r.h>
#include <libsc/simple_buzzer.h>
#include <string>
#include <libsc/dir_motor.h>
#include <libsc/lcd_console.h>
#include <libsc/lcd_typewriter.h>
#include <libsc/battery_meter.h>
#include <cstring>
#include <sstream>
#include <functional>

#define GetL(x) x
#define GetR(x) x+60

class Car {
	inline bool GetPixel(const Byte* src, const int8_t x, const int8_t y) {
		//	const int offset = x/8 + (y * image_width / 8);
		//	return (src[offset] << (x%8) & 0x80) ? false : true;
		return !(src[x / 8 + (y * 80 / 8)] << (x % 8) & 0x80);
	}
public:
	// public means anyone can access it
	//there are even , that means really anyone can access it, even when the class object does not exist :O
	// if you wanna do that, just search " class friend c++"
	Car();
	// default constructor, it will be automatically called once the class object is created
	// example : Car D7689;
	// once the compiler run this code, Car() will be immediately called.
	//same name as the class name
	~Car();
	//default destructor, it will be automatically called when the class object is gonna out the scope
	// example : int main(){  while(1){Car D7689;}   }
	// in the example, D7689 is inside the scope of the while loop, that means after ' ; 'destructor will be called
	// in other word you will D 7 689 again and again
	// if u still dont understand, google "class c++" :)
	//same name as the class name with a '~'

	/*--------------------------------printing below------------------------------------*/

	void printvalue(int16_t value, int16_t color);
	//print number to up-right corner in LCD

	void printvalue(int x, int y, int w, int h, int16_t value, int16_t color);//my favorite function!! Nicely done!
	//print number to specific location, (x,y,w,h,value-to-print) respectively

	void printvalue(std::string);
	//print string to up-right corner in LCD

	void printvalue(int x, int y, int w, int h, std::string Result,
			int16_t color);

	void printRawCamGraph(const int8_t x, const int8_t y, Byte* data);
	//call after capture_raw_image

	void printEdge(const int8_t xpos, const int8_t ypos);

	void printWaypoint(const int8_t xpos, const int8_t ypos);

	void clearLcd(uint16_t);
	//clear the lcd the the given color
	// its very slow

	void printline(int16_t value, uint16_t color);
	// print a horizontal line, could be for threshold

	/*--------------------------------printing above------------------------------------*/

	/*--------------------------------signal component below------------------------------------*/

	void blinkLED(int8_t id, int delay_time, int persist_time);
	//blink LED
	//Internal delay inside

	void beepbuzzer(uint32_t);
	//beep buzzer for t (ms)

	void switchLED(int8_t id);
	void switchLED(int8_t id, bool isEnable);
	/*--------------------------------signal component above------------------------------------*/

	/*--------------------------------get data from component below------------------------------------*/

	void capture_image(void);
	//capture and divide image,change data AND image

	/*--------------------------------get data from component above------------------------------------*/

	/*--------------------------------(temporarily) get devices below------------------------------------*/

	libsc::St7735r& GetLCD(void) {
		return *LCD;
	}

	/*--------------------------------(temporarily) get devices above------------------------------------*/

	/*--------------------------------implement in inherited class------------------------------------*/
	//virtual simply means you can have different implementation of that function in the inherited class
	// dont understand? NVM, just skip this part
	virtual void turningPID(int8_t const mid_line, const float, const float,const uint8_t)=0;
	//positional PID = kp *error +kd *(error_prev - error), try change Kp according to error magnitude

	virtual void motorPID(const int16_t ideal_encoder_count, const float,
			const float, const float, const float,const float)=0;
	// Incremental PID(n) = PID(n-1) + kp * (e(n)-e(n-1)) +kd *(e(n)-2e(n-1)+e(n-2)) + ki * e(n)
	// which means previous PID, two of the previous errors should be cached

	/*------data containers below------*/
	//containers for data to bypass fucking C pointers
	//data container for direct camera data extraction
	Byte data[600];

	//image divided into pixels
	bool image[80][60];

	//this is pretty self-explanatory, note that first half left, second half right.*I HATE POINTERS!*
	int8_t edges[120];

	int8_t waypoints[60];

	//store at which y coordinate the bg/obs starts, to utilize resources
	int8_t bgstart;

	int mid;	//stupid compiler always overflow when I reduce the size

	/*------data containers above-------*/

private:
	// private means only member of it's only class can access it.

	//function used to divide image,

protected:
	//protected seems professional
	//protected means either member of it's only class or class inherited

	libsc::Led* Led1 = nullptr;
	libsc::Led* Led2 = nullptr;
	libsc::Led* Led3 = nullptr;
	libsc::Led* Led4 = nullptr;
#ifndef Use_Dir_Encoder
	libsc::AbEncoder* encoder = nullptr;
#else
	libsc::DirEncoder* encoder = nullptr;
#endif
	libsc::TrsD05* servo = nullptr;
	libsc::DirMotor* motor = nullptr;
	libsc::St7735r* LCD = nullptr;
	libsc::LcdConsole* LCDconsole = nullptr;
	libsc::LcdTypewriter* LCDwriter = nullptr;
	libsc::SimpleBuzzer* buzzer = nullptr;
	libsc::k60::Ov7725* cam = nullptr;
	libsc::BatteryMeter* batt = nullptr;

	//the above * means its a pointer.
	// either  int* a;
	// or 	   int *a;
	// are correct, but int* a is recommended. The reason is....i forgot, just better to do this

};
