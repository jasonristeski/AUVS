#pragma once
#include "opencv2/core.hpp"

struct BeaconData
{
	cv::Rect rect;
	// only Hue is tracked and updated, other values are set to Min,Max for their respected ranges
	cv::Vec3f HSV;
	cv::Vec3f range; // initially const value 10,255,255;
	std::vector<cv::Vec3f> history;
	std::string ID;
};
