/*
 * ImageProcess.cpp
 *
 *  Created on: 1 Apr 2016
 *      Author: Kyle, Judy
 *  Copyright © 2015-2016 HKUST SmartCar Team. All rights reserved.
 */

#include "../inc/ImageProcess.h"

#include <algorithm>
#include <cstring>

#define CONTINUOUS 5
#define THRES 10
#define CAMW 80
#define CAMH 60
#define recL(y) y
#define recR(y) y+60

void ImageProcess::FindEdge(const Byte* data, int8_t edges[120],
		int8_t waypoints[60], int8_t& m_bgstart, const int8_t thres,
		const int8_t offset, bool& stop) {

//	std::memset(edges, 0, 60);
//	std::memset(edges + 60, 79, 60);
	int8_t lastLeft = 0;
	int8_t lastRight = 0;
	int8_t last2Left = 0;
	int8_t last2Right = 0;
	bool crossroad = false;
	m_bgstart = 0;

	/*-----find bottom left-----*/
	for (int8_t x = 0; x < CAMW; x++) {
		if (GetPixel(data, x, CAMH - 1)) {
			lastLeft = x;
			last2Left = x;
			edges[recL(CAMH-1)] = x;
			break;
		}
	}

	/*-----find bottom right-----*/
	for (int8_t x = CAMW - 1; x > lastLeft; x--) {
		if (GetPixel(data, x, CAMH - 1)) {
			lastRight = x;
			last2Right = x;
			edges[recR(CAMH-1)] = x;
			break;
		}
	}
	waypoints[CAMH - 1] = (edges[recL(CAMH-1)] + edges[recR(CAMH-1)]) / 2;

	/*-----scan every row from bottom up-----*/
	for (int8_t y = CAMH - 2; y >= 0; y--) {

		rowstart: ;
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

		/*---cross road processing---*/
		switch (m_xMode) {
		case VarSet::CrossroadMode::kAllWhite: {
			if (!crossroad && y >= CONTINUOUS && y <= 50) {
				/*-----all white rows-----*/
				bool all_white = true;
				for (int8_t i = THRES; i < CAMW - THRES; i++)
					if (!GetPixel(data, i, y)) {
						all_white = false;
						break;
					}
				if (all_white) {
					bool all_white1 = true;
					for (int8_t i = THRES; i < CAMW - THRES; i++)
						for (int8_t j = 1; j < CONTINUOUS; j++)
							if (!GetPixel(data, i, y - j)) {
								all_white1 = false;
								break;
							}
					if (all_white1)
						crossroad = true;
				}
				/*-----add imaginary lines when crossroad detected-----*/
				if (crossroad) {
					for (int i = CAMH - 5; i >= 0; i--) {
						if (edges[recL(CAMH-1)] != 0) {
							if (edges[recL(CAMH-1)] < edges[CAMH - 5])
								edges[recL(i)] = (edges[recL(CAMH-5)]
										- edges[recL(CAMH-1)]) * (CAMH - 5 - i)
										/ 4 + edges[recL(CAMH-5)];
							else
								edges[recL(i)] = (CAMH - 1 - i) / 3
										+ edges[recL(CAMH-1)];
						}
						if (edges[recR(CAMH-1)] != CAMW - 1) {
							if (edges[recR(CAMH-1)] > edges[CAMH - 5])
								edges[recR(i)] = (edges[recR(CAMH-5)]
										- edges[recR(CAMH-1)]) * (CAMH - 5 - i)
										/ 4 + edges[recR(CAMH-5)];
							else
								edges[recR(i)] = (CAMH - 1 - i) / 3
										+ edges[recR(CAMH-1)];

						}
						edges[recL(i)] =
								edges[recL(i)] < 0 ? 0 : edges[recL(i)];
						edges[recR(i)] =
								edges[recR(i)] > 79 ? 79 : edges[recR(i)];
						int8_t mid = (edges[recL(i)] + edges[recR(i)]) / 2;
						waypoints[i] = mid;
					}
					goto end;
				}
			}
		}
			break;

		case VarSet::CrossroadMode::kOutwards: {
			if (y < CAMH - 4) {
				if (edges[recL(y)] - edges[recL(y + 2)] < 0
						&& edges[recR(y)] - edges[recR(y + 2)] > 0) { //both sides outwards, add both sides
					edges[recL(y)] = 2 * edges[recL(y + 2)]
							- edges[recL(y + 3)];
					edges[recR(y)] = 2 * edges[recR(y + 2)]
							- edges[recR(y + 3)];
				} else {
					if (edges[recL(y + 2)] > edges[recL(y + 3)]
							&& edges[recL(y)] < edges[recL(y + 2)])
						edges[recL(y)] = 2 * edges[recL(y + 2)]
								- edges[recL(y + 3)];
					if (edges[recR(y + 2)] < edges[recR(y + 3)]
							&& edges[recR(y)] > edges[recR(y + 2)])
						edges[recR(y)] = 2 * edges[recR(y + 2)]
								- edges[recR(y + 3)];
				}
			}
			if (--y >= 0)
				goto rowstart;
		}
			break;

		case VarSet::CrossroadMode::kLazy: {
			if (!crossroad && y >= CONTINUOUS && y <= 50) {
				bool all_white = true;
				for (int8_t i = THRES; i < CAMW - THRES; i++)
					if (!GetPixel(data, i, y)) {
						all_white = false;
						break;
					}
				if (all_white) {
					bool all_white1 = true;
					for (int8_t i = THRES; i < CAMW - THRES; i++)
						for (int8_t j = 1; j < CONTINUOUS; j++)
							if (!GetPixel(data, i, y - j)) {
								all_white1 = false;
								break;
							}
					if (all_white1) {
						crossroad = true;
						bool tilted = false;
						for (int8_t i = y + 1; i < CAMH - 1; i++) {
							if (edges[recL(i)] < edges[recL(i - 1)]
									|| edges[recR(i)] > edges[recR(i - 1)]) {
								tilted = true;
								m_bgstart = i - 1;
								goto end;
							}
						}
						if (!tilted)
							m_bgstart = y;
						goto end;
					}
				}
			}
		}
		}

		/*------stop condition------*/
		if (y > CAMH - 15) {
			int8_t countsr = 0, countsl = 0;
			bool rightblack1 = false;
			bool leftblack1 = false;
			bool leftwhite = false;
			bool rightwhite = false;
			int8_t middle = (edges[recL(y)] + edges[recR(y)]) / 2;
			if (GetPixel(data, middle, y)) {
				for (int8_t i = middle; i < edges[recR(y)]; i++) {
					if (!GetPixel(data, i, y))
						countsr++;
					if (countsr > 3) {
						rightblack1 = true;
						break;
					}
				}
			}
			if (rightblack1) {
				for (int8_t i = middle; i > edges[recL(y)]; i--) {
					if (!GetPixel(data, i, y))
						countsl++;
					if (countsl > 3 && !crossroad) {
						leftblack1 = true;
						break;
					}
				}
			}
			if (leftblack1) {
				for (int8_t i = edges[recL(y)] + 1; i < edges[recL(y)] + 4; i++)
					if (GetPixel(data, i, y)) {
						leftwhite = true;
						break;
					}
			}
			if (leftwhite) {
				for (int8_t i = edges[recR(y)] - 1; i > edges[recL(y)] - 4; i--)
					if (GetPixel(data, i, y)) {
						rightwhite = true;
						break;
					}
			}
			if (rightwhite) {
				for (int8_t i = y; i < CAMH -10 ; i++) {
					if (GetPixel(data, 0, i) || GetPixel(data, 79, i))
						break;
					if (i == CAMH - 11) {
						stop = true;
						goto end;
					}
				}
			}

		}
		stop = false;

		/*-----store variables for future processing-----*/
		last2Left = lastLeft;
		last2Right = lastRight;
		lastLeft = edges[recL(y)];
		lastRight = edges[recR(y)];

		/*-----start finding obstacle-----*/
		bool found = false;
		/*-----scan from center to right to find obstacle-----*/
		for (int8_t x = (edges[recL(y)] + edges[recR(y)]) / 2;
				x < (edges[recL(y)] + 3 * edges[recR(y)]) / 4; x++) {
			if (!GetPixel(data, x, y)) {
				if (!IsNoise(data, x, y)) {
					edges[recR(y)] = x - offset;
					found = true;
					break;
				}
			}
		}

		/*-----if not found, scan from center to left to find obstacle-----*/
		if (!found) {
			for (int8_t x = (edges[recL(y)] + edges[recR(y)]) / 2;
					x > (3 * edges[recL(y)] + edges[recR(y)]) / 4; x--) {
				if (!GetPixel(data, x, y)) {
					if (!IsNoise(data, x, y)) {
						edges[recL(y)] = x + offset;
						break;
					}
				}
			}
		}

		waypoints[y] = (edges[recL(y)] + edges[recR(y)]) / 2;

		/*-----finish scanning-----*/
	} //every row
	end: ; //finish scanning full frame

}

inline bool ImageProcess::IsNoise(const Byte* data, const int8_t m_x,
		const int8_t m_y) {
	if (m_x >= 1 && m_x <= CAMW - 2 && m_y >= 1 && m_y <= CAMH - 2) {
		if (GetPixel(data, m_x - 1, m_y) && GetPixel(data, m_x + 1, m_y)
				&& GetPixel(data, m_x, m_y - 1)
				&& GetPixel(data, m_x, m_y + 1)) {
			return true;
		} else
			return false;
	} else
		return false;
}
