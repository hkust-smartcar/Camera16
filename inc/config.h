/*
 * config.h
 *
 *  Created on: 28-3-2016
 *      Author: Kyle
 *      Adapted code written by yungc
 *  Copyright © 2015-2016 HKUST SmartCar Team. All rights reserved.
 */

#pragma once

//put all config here

#include <functional>
#include <libsc/system.h>
#include <libsc/led.h>
#include <libbase/k60/mcg.h>
#include <libsc/ab_encoder.h>
#include <libsc/trs_d05.h>
#include <libsc/k60/ov7725.h>
#include <libsc/button.h>
#include <libsc/joystick.h>
#include <libsc/st7735r.h>
#include <libsc/simple_buzzer.h>
#include <libsc/dir_motor.h>
#include <libsc/battery_meter.h>

using namespace libsc;

namespace libbase
{
	namespace k60
	{

		Mcg::Config Mcg::GetMcgConfig()
		{
			Mcg::Config config;
			config.external_oscillator_khz = 50000;
			config.core_clock_khz = 150000;
			return config;
		}

	}
}

//LED 1~4
Led::Config GetLed1Config()
{
	Led::Config Led1Config;
	Led1Config.id = 0;
	Led1Config.is_active_low = true;
	return Led1Config;
}
Led::Config GetLed2Config()
{
	Led::Config Led2Config;
	Led2Config.id = 1;
	Led2Config.is_active_low = true;
	return Led2Config;
}
Led::Config GetLed3Config()
{
	Led::Config Led3Config;
	Led3Config.id = 2;
	Led3Config.is_active_low = true;
	return Led3Config;
}
Led::Config GetLed4Config()
{
	Led::Config Led4Config;
	Led4Config.id = 3;
	Led4Config.is_active_low = true;
	return Led4Config;
}

//encoder
AbEncoder::Config GetAbEncoderConfig()
{
	AbEncoder::Config EncoderConfig;
	EncoderConfig.id = 0;
	return EncoderConfig;
}

//servo for turning
TrsD05::Config GetServoConfig()
{
	TrsD05::Config ServoConfig;
	ServoConfig.id = 0;
	return ServoConfig;
}

DirMotor::Config GetDirmotorConfig()
{
	DirMotor::Config DirmotorConfig;
	DirmotorConfig.id=0;
	return DirmotorConfig;
}

St7735r::Config GetLcdConfig()
{
	St7735r::Config LcdConfig;
	return LcdConfig;
}

SimpleBuzzer::Config GetBuzzerConfig()
{
	SimpleBuzzer::Config BuzzerConfig;
	BuzzerConfig.id = 0;
	BuzzerConfig.is_active_low = false;
	return BuzzerConfig;
}

k60::Ov7725::Config GetCamConfig(){
	k60::Ov7725::Config camcfg;
	camcfg.w=80;
	camcfg.h=60;
	camcfg.fps=k60::Ov7725::Config::Fps::kMid;
	//TODO adjust appropriate FPS
	camcfg.contrast = 0x30;
//	camcfg.brightness=0x00;
	return camcfg;
}

