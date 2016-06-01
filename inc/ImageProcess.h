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

class ImageProcess {
	//C++11 so OP, even allow direct passing of 2D array!
public:
	ImageProcess() {};
	~ImageProcess() {};

	/*------processing below------*/

	//Slope-Oriented Edge Detecting Algorithm
	//take[80][60], process and put into[120]. first half x-coordinates of left edge, second half x-coordinates of right edge
	//store where background start in bgstart, to prevent resources loss
	//MUST pass by reference(&), otherwise bgstart will NEVER change!
	void FindEdge(const bool m_image[80][60], int8_t m_edges[120],
			int8_t& m_bgstart, const int8_t thres, const int8_t offset);

	/*------processing above------*/

private:

};
