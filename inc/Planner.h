/*
 * Planner.h
 *
 *  Created on: 4 Apr 2016
 *      Author: Kyle
 *	Copyright © 2015-2016 HKUST SmartCar Team. All rights reserved.
 *
 *   Outputs ideal motor speed and servo error, based on weighed average of the road info.
 */
#pragma once
#include "RunMode.h"

class Planner{
public:
	Planner(const int8_t starting_row);
	~Planner(void);

	//simply calculate average of edges, and put into waypoints[60]
	void Calc(const int8_t waypoints[60],int8_t const bgstart,int32_t& mid);
	void ChangeWeight(const int8_t starting_row);

private:
	uint weight[60];
//	uint8_t status;
};
