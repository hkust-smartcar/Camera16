/*
 * ImageProcess.cpp
 *
 *  Created on: 1 Apr 2016
 *      Author: Kyle
 *  Copyright © 2015-2016 HKUST SmartCar Team. All rights reserved.
 */

#include "../inc/ImageProcess.h"

#include <algorithm>
#include <cstring>

#define CAMW 80
#define CAMH 60
#define recL(y) y
#define recR(y) y+60

void ImageProcess::FindEdge(const bool image[80][60], int8_t edges[120],
		int8_t& m_bgstart, const int8_t thres, const int8_t offset,
		bool& stop) {

	std::memset(edges, 39, 120);
	int8_t lastLeft = 0;
	int8_t lastRight = 0;
	int8_t last2Left = 0;
	int8_t last2Right = 0;
	int8_t last3Left = 0;
	int8_t last3Right = 0;
	bool crossroad=false;

	/*-----find bottom left-----*/

	for (int8_t x = 0; x < CAMW; x++) {
		if (image[x][CAMH - 1]) {
			lastLeft = x;
			last2Left = x;
			last3Left = x;
			edges[recL(CAMH-1)] = x;
			break;
		}
	}

	/*-----find bottom right-----*/

	for (int8_t x = CAMW - 1; x > lastLeft; x--) {
		if (image[x][CAMH - 1]) {
			lastRight = x;
			last2Right = x;
			last3Right = x;
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

		/*------stop condition by Judy------*/
		if (y > CAMH - 5) {
			bool rightfulfill = false;
			int8_t middle = (edges[recL(y)] + edges[recR(y)]) / 2;
			if (image[middle][y])
				for (int8_t i = middle; i < edges[recR(y)]; i++)
					if (!image[i][y]) {
						for (int8_t j = i; j < edges[recR(y)]; j++)
							if (image[j][y]) {
								rightfulfill = true;
								goto left;
							}
					}
			left: if (rightfulfill) {
				for (int8_t i = middle; i > edges[recL(y)]; i--)
					if (!image[i][y]) {
						for (int8_t j = i; j > edges[recL(y)]; j--)
							if (image[j][y]) {
								stop = true;
								goto end;
							}
					}
			}
		}
		stop = false;

		if (y > 30) {
			if (edges[recL(y)] <= 0 && edges[recR(y)] >= CAMW - 1) {
				if (lastLeft <= 0 && lastRight >= CAMW - 1)
					if (last2Left <= 0 && last2Right >= CAMW - 1)
						if (last3Left <= 0 && last3Right >= CAMW - 1) {
							crossroad = true;
						}

			}
		}

		if (crossroad) {
			edges[recL(y)] = lastLeft + last2Left - last3Left;
			edges[recR(y)] = lastRight + last2Right - last3Right;
		}
//
//		else if (last2Left - last3Left > 0 && edges[recL(y)] - last2Left < 0)
//			edges[recL(y)] = lastLeft + last2Left - last3Left;
//		else if (last2Right - last3Right < 0 && edges[recR(y)] - last2Right > 0)
//			edges[recR(y)] = lastRight + last2Right - last3Right;

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
		/*-----store variables for future processing-----*/
		last3Left = last2Left;
		last3Right = last2Right;
		last2Left = lastLeft;
		last2Right = lastRight;
		lastLeft = edges[recL(y)];
		lastRight = edges[recR(y)];

	}
	end: ;

}
