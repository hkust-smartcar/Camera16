/*
 * Planner.cpp
 *
 *  Created on: 2 Apr 2016
 *      Author: Kyle
 * 	Copyright © 2015-2016 HKUST SmartCar Team. All rights reserved.
 *
 */
#include <Planner.h>
#include <car.h>
#define CAMH 60

Planner::Planner() {
	for (int8_t row = CAMH - 1; row >= 0; row--) {
		weight[row] = row * row; // define the weight for each item
	}
}
void Planner::Calc(int8_t const edge[120], int8_t waypoints[60],
		int8_t const bgstart, uint& mid) {

	uint m_mid_sum = 0;
	uint m_weight_sum = 0;

	for (int8_t y = CAMH - 1; y > bgstart; y--) {
		waypoints[y] = (edge[GetL(y)] + edge[GetR(y)]) / 2; //TODO: try adding imaginary lines here
		m_mid_sum += waypoints[y] * weight[y];
		m_weight_sum += weight[y];
	}

	mid = m_mid_sum / m_weight_sum;
}

