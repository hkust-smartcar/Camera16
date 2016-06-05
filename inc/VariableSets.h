/*
 * VariableSets.h
 *
 *  Created on: 1 Jun 2016
 *      Author: leizhao
 *
 *  Store different sets of fine-tuned variables (Kp, Ki...) for main to load
 */

#pragma once

#include <stdint.h>

struct VarSet {
	int16_t ideal_encoder_count;
	/*-----servo-----*/
	float T; //kp
	float K; //kd

	/*-----motor-----*/
	float Kp;
	float Ki;
	float Kd;

	/*-----other processing variables-----*/
	int8_t offset;
	int8_t plnstart;
};
