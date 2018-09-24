#pragma once
#include "opencv2/core.hpp"

#include "BeaconData.h"
#include <vector>

class BeaconDetector
{
public:
	BeaconDetector();
	~BeaconDetector();
	std::vector<BeaconData> GetDetectedBeacons();
	void Detect(cv::Mat frame, std::vector<BeaconData> readings);
private:
	void Convert(cv::Mat& frame);
	void Morph(cv::Mat& frame);
	void ValidateBounds(cv::Vec3f& lower, cv::Vec3f& upper);
	cv::Vec3f ExtractHSV(cv::Rect pt);

	cv::Mat ColorThreshold(cv::Mat frame, cv::Vec3f& lower, cv::Vec3f& upper);

	std::vector<cv::Rect> CannySegment(cv::Mat& frame, int minArea, int maxArea);
	cv::Rect Search(std::vector<cv::Rect>& data, BeaconData& previousReading);


	cv::Vec3f Sample(cv::Rect window,cv::Vec3f& meanHSV);
	cv::Vec3f Sample(cv::Rect window);


	float HSVDiff(cv::Vec3f& mean, cv::Vec3f& sample);
	float SizeDiff(cv::Rect& potential, cv::Rect& previous);
	float PosDiff(cv::Rect& potential, cv::Rect& previous);
	float GetFitScore(cv::Rect& data, BeaconData& prevoiusReading);
	float GetFitScore(BeaconData& newReading, BeaconData& previousReading);

	void ScentedDetection(BeaconData& data);
	void UnscentedDetection(BeaconData& data);

	cv::Mat rawFrame, HSVFrame;
	std::vector<BeaconData> beacons;

};

