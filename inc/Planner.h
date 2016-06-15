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
#include "car.h"

class Planner{
public:
	Planner(void);
	~Planner(void);

	//simply calculate average of edges, and put into waypoints[60]
	void Calc(int8_t const edge[120],int8_t waypoints[60],int8_t const bgstart,int32_t& mid);

private:
	uint weight[60];
//	uint8_t status;
};
