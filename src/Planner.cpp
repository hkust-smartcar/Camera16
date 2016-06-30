/*
 * Planner.cpp
 *
 *  Created on: 2 Apr 2016
 *      Author: Kyle
 * 	Copyright © 2015-2016 HKUST SmartCar Team. All rights reserved.
 *
 */

#include "../inc/Planner.h"

#include <sys/types.h>
#include <cstdint>
#include <cmath>

#define CAMH 60

Planner::Planner(VarSet& m_VarSet) {
	switch (m_VarSet.mode) {
	case VarSet::PlannerMode::kProportional:
		for (int8_t row = CAMH - 1; row >= 0; row--) {
			weight[row] = row; // define the weight for each item
		};
		break;
	case VarSet::PlannerMode::kRoot:
		for (int8_t row = CAMH - 1; row >= 0; row--) {
			weight[row] = sqrt(row); // define the weight for each item
		};
		break;
	case VarSet::PlannerMode::kSquared:
		for (int8_t row = CAMH - 1; row >= 0; row--) {
			weight[row] = row * row; // define the weight for each item
		};
		break;
	}
}

Planner::~Planner() {
}
void Planner::Calc(const int8_t waypoints[60], int8_t const bgstart,
		int32_t& mid) {

	int m_mid_sum = 0;
	int m_weight_sum = 0;

	for (int8_t y = CAMH - 1; y > bgstart; y--) {
		m_mid_sum += waypoints[y] * weight[y];
		m_weight_sum += weight[y];
	}

	mid = m_mid_sum / (m_weight_sum == 0 ? 1 : m_weight_sum);
	if (mid == 0)
		mid = 39;
}

