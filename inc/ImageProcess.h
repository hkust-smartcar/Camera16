/*
 * ImageProcess.h
 *
 * Author: Kyle
 * Copyright (c) 2015-2016 HKUST SmartCar Team
 * Refer to LICENSE for details
 *
 * Handle edge and obstacles searching
 */
#pragma once
#include <stdint.h>
#include <RunMode.h>
struct VarSet;

class ImageProcess {
	inline bool GetPixel(const Byte* src, const int8_t x, const int8_t y) {
		//	const int offset = x/8 + (y * image_width / 8);
		//	return (src[offset] << (x%8) & 0x80) ? false : true;
		return !(src[x / 8 + (y * 80 / 8)] << (x % 8) & 0x80);
	}
public:
	ImageProcess(VarSet& m_VarSet){
		m_xMode=m_VarSet.xMode;
	}
	~ImageProcess() {};

	//Slope-Oriented Edge Detecting Algorithm
	//take[80][60], process and put into[120]. first half x-coordinates of left edge, second half x-coordinates of right edge
	//store where background start in bgstart, to prevent waste of resources
	//MUST pass by reference(&), otherwise bgstart will NEVER change!
	void FindEdge(const Byte* data, int8_t m_edges[120],int8_t waypoints[60],
			int8_t& m_bgstart, const int8_t thres, const int8_t offset,
			bool& stop);
private:
	VarSet::CrossroadMode m_xMode;
};
