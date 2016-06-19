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

void ImageProcess::FindEdge(const Byte* data, int8_t edges[120],
		int8_t waypoints[60], int8_t& m_bgstart, const int8_t thres,
		const int8_t offset, bool& stop) {

	std::memset(edges, 0, 60);
	std::memset(edges + 60, 79, 60);
	int8_t lastLeft = 0;
	int8_t lastRight = 0;
	int8_t last2Left = 0;
	int8_t last2Right = 0;
//	int8_t last3Left = 0;
//	int8_t last3Right = 0;
	bool crossroad = false;
	m_bgstart = 0;

	/*-----find bottom left-----*/
	for (int8_t x = 0; x < CAMW; x++) {
		if (GetPixel(data, x, CAMH - 1)) {
			lastLeft = x;
			last2Left = x;
//			last3Left = x;
			edges[recL(CAMH-1)] = x;
			break;
		}
	}

	/*-----find bottom right-----*/
	for (int8_t x = CAMW - 1; x > lastLeft; x--) {
		if (GetPixel(data, x, CAMH - 1)) {
			lastRight = x;
			last2Right = x;
//			last3Right = x;
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
			if (GetPixel(data, x, y)) {
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
			if (GetPixel(data, x, y)) {
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
			if (GetPixel(data, middle, y))
				for (int8_t i = middle; i < edges[recR(y)]; i++)
					if (!GetPixel(data, i, y)) {
						for (int8_t j = i; j < edges[recR(y)]; j++)
							if (GetPixel(data, j, y)) {
								rightfulfill = true;
								goto left;
							}
					}
			left: if (rightfulfill) {
				for (int8_t i = middle; i > edges[recL(y)]; i--)
					if (!GetPixel(data, i, y)) {
						for (int8_t j = i; j > edges[recL(y)]; j--)
							if (GetPixel(data, j, y)) {
								stop = true;
								goto end;
							}
					}
			}
		}
		stop = false;

		/*---find cross road---*/
		if (!crossroad) {
			bool all_white = true;
			for (int8_t i = 0; i < CAMW; i++)
				if (!GetPixel(data, i, y - 1)) {
					all_white = false;
					break;
				}
			if (all_white) {
				bool all_white1 = true;
				for (int8_t i = 0; i < CAMW; i++)
					if (!GetPixel(data, i, y - 2)
							|| !GetPixel(data, i, y - 3)) {
						all_white1 = false;
						break;
					}
				if (all_white1)
					crossroad = true;
				edges[recL(y)] = (lastLeft - edges[recL(59)]) * (y - 59)
						/ (y - 58) + edges[recL(59)];
				edges[recR(y)] = (lastRight - edges[recR(59)]) * (y - 59)
						/ (y - 58) + edges[recR(59)];
			}
		}

		if (crossroad && y < CAMH - 3) { //first 3 rows cannot be trusted, since they are initialized following the bottom row
			edges[recR(y)] = 2 * lastRight - last2Right;
			edges[recL(y)] = 2 * lastLeft - last2Left;
		}
		/*-----start finding obstacle-----*/
		bool found = false;

		int8_t mid = (edges[recL(y)] + edges[recR(y)]) / 2;
		waypoints[y] = mid;
		/*-----scan from center to right to find obstacle-----*/
		for (int8_t x = mid; x < edges[recR(y)]; x++) {
			if (!GetPixel(data, x, y)) {
				edges[recR(y)] = x - offset;
				found = true;
				break;
			}
		}

		/*-----if not found, scan from center to left to find obstacle-----*/
		if (!found) {
			for (int8_t x = mid; x > edges[recL(y)]; x--) {
				if (!GetPixel(data, x, y)) {
					edges[recL(y)] = x + offset;
					break;
				}
			}
		}
		/*-----finish scanning-----*/
		/*-----store variables for future processing-----*/
//		last3Left = last2Left;
//		last3Right = last2Right;
		last2Left = lastLeft;
		last2Right = lastRight;
		lastLeft = edges[recL(y)];
		lastRight = edges[recR(y)];

	} //every row
	end: ; //finish scanning full frame

}
