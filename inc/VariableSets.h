/*
 * VariableSets.h
 *
 *  Created on: 1 Jun 2016
 *      Author: leizhao
 *
 *  Store different sets of fine-tuned variables (Kp, Ki...) for main to load
 */

#pragma once

#include <libbase/k60/watchdog.h>
#include <libsc/lcd.h>
#include <libsc/system.h>
#include <stdint.h>

#include "RunMode.h"

struct VarSet {
	/*-----servo-----*/
	float K;
	float T;

	/*-----motor-----*/
	float Kp;
	float Ki;
	float Kd;

	/*-----other processing variables-----*/
	int8_t offset;
	int8_t plnstart;
};
