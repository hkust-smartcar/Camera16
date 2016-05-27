/*
 * ImageProcess.cpp
 *
 *  Created on: 1 Apr 2016
 *      Author: Kyle
 *  Copyright © 2015-2016 HKUST SmartCar Team. All rights reserved.
 */

#include "../inc/ImageProcess.h"

#include <stdint.h>
#include <algorithm>

#define CAMW 80
#define CAMH 60
#define recL(y) y
#define recR(y) y+60

void ImageProcess::FindEdge(const bool image[80][60], int8_t edges[120],
		int8_t& m_bgstart, const int8_t thres, const int8_t offset) {

	int8_t lastLeft = 0;
	int8_t lastRight = 0;
	int8_t last2Left = 0; //compute slope to estimate thres
	int8_t last2Right = 0;

	/*-----find bottom left-----*/

	for (int8_t x = 0; x < CAMW; x++) {
		if (image[x][CAMH - 1]) {
			lastLeft = x;
			last2Left = x;
			edges[recL(CAMH-1)] = x;
			break;
		}
	}

	/*-----find bottom right-----*/

	for (int8_t x = CAMW - 1; x > lastLeft; x--) {
		if (image[x][CAMH - 1]) {
			lastRight = x;
			last2Right = x;
			edges[recR(CAMH-1)] = x;
			break;
		}
	}

	/*-----scan every row from bottom up-----*/

	for (int8_t y = CAMH - 2; y >= 0; y--) {

		bool leftFound = false;
		bool rightFound = false;

		/*-----scan from left according to estimated slope-----*/
		for (int8_t x = std::max(0, 2 * lastLeft - last2Left - thres);
				x < std::min(CAMW - 1, 2 * lastLeft - last2Left + thres); x++) {
			if (image[x][y]) {
				edges[recL(y)] = x;
				leftFound = true;
				break;
			}
//			else
//				edges[recL(y)] = std::max(0, 2 * lastLeft - last2Left);
		}

		/*-----scan from right according to estimated slope-----*/
		for (int8_t x = std::min(CAMW - 1, 2 * lastRight - last2Right + thres);
				x > std::max(0, 2 * lastRight - last2Right - thres); x--) {
			if (image[x][y]) {
				edges[recR(y)] = x;
				rightFound = true;
				break;
			}
//			else
//				edges[recR(y)] = std::min(CAMW - 1, 2 * lastRight - last2Right);
		}

		/*-----if neither side have been found, assert that the bg starts this row and break the loop-----*/
		if (!leftFound && !rightFound) {
			m_bgstart = y;
			break;
		}

		/*-----store variables for future processing-----*/
		last2Left = lastLeft;
		last2Right = lastRight;
		lastLeft = edges[recL(y)];
		lastRight = edges[recR(y)];

		/*-----start finding obstacle-----*/
		bool found = false;

		/*-----scan from center to right to find obstacle-----*/
		for (int8_t x = (edges[recL(y)] + edges[recR(y)]) / 2;
				x < edges[recR(y)]; x++) {
			if (!image[x][y]) {
				edges[recR(y)] = x - offset;
				found = true;
				break;
			}
		}

		/*-----if not found, scan from center to left to find obstacle-----*/
		if (!found) {
			for (int8_t x = (edges[recL(y)] + edges[recR(y)]) / 2;
					x > edges[recL(y)]; x--) {
				if (!image[x][y]) {
					edges[recL(y)] = x + offset;
					break;
				}
			}

		}
		/*-----finish scanning-----*/
	}
}
