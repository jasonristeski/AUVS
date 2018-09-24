#include "BeaconDetector.h"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/features2d.hpp"


#include <time.h>
#include <iostream>
int MIN_AREA = 150;
int MAX_AREA = 10000;
int SAMPLE_SIZE = 15;
int MAX_ITER = 150;


float a = 2;
float b = 4;
float c = 3;

int searchWindow = 2;
int counter = 0;
BeaconDetector::BeaconDetector()
{
	srand(time(NULL));
}


BeaconDetector::~BeaconDetector()
{
}

std::vector<BeaconData> BeaconDetector::GetDetectedBeacons()
{
	return beacons;
}

float BeaconDetector::HSVDiff(cv::Vec3f& previousReading, cv::Vec3f& sample)
{
	return std::min(std::abs(sample[0] - previousReading[0]), 180 - std::abs(sample[0] - previousReading[0])) / 90;
}

float BeaconDetector::SizeDiff(cv::Rect& previousReading, cv::Rect& sample)
{
	float heightDiff = std::abs(sample.height - previousReading.height);
	float widthDiff = std::abs(sample.width - previousReading.width);
	return heightDiff * widthDiff;
}

float BeaconDetector::PosDiff(cv::Rect& previous, cv::Rect& sample)
{
	cv::Point2f a(previous.x + previous.width * 0.5, previous.y + previous.height * 0.5);
	cv::Point2f b(sample.x + sample.width * 0.5, sample.y + sample.height * 0.5);
	cv::Mat display = rawFrame.clone();
	cv::circle(display, a, 1, cv::Scalar(0, 255, 255), 1, 8, 0);
	cv::rectangle(display, previous, cv::Scalar(100, 255, 255), 1, 8, 0);
	cv::circle(display, b, 1, cv::Scalar(0, 255, 0), 1, 8, 0);
	cv::imshow("POS DIFF", display);
	return sqrt(((a.x - b.x)*(a.x - b.x)) + ((a.y - b.y)*(a.y - b.y)));
}

float BeaconDetector::GetFitScore(cv::Rect& data, BeaconData& prevoiusReading)
{
	float size = SizeDiff(prevoiusReading.rect, data);
	float pos = PosDiff(prevoiusReading.rect, data);
	cv::Vec3f H = Sample(data);
	float color = HSVDiff(prevoiusReading.HSV, H);
	
	
	//std::cerr <<  "SCORE VARAIBLES "  <<size << " " << pos << " " << color << " " << std::endl;
	return a * size + b * pos + c * color;
}

float BeaconDetector::GetFitScore(BeaconData& newReading, BeaconData& previousReading)
{
	float size = SizeDiff(previousReading.rect, newReading.rect);
	float pos = PosDiff(previousReading.rect, newReading.rect);
	float color = HSVDiff(previousReading.HSV, newReading.HSV);
	return (a * size) + (b * pos) + (c * color);

}

void BeaconDetector::Convert(cv::Mat& frame)
{
	frame.copyTo(rawFrame);
	frame.copyTo(HSVFrame);
	cv::cvtColor(HSVFrame, HSVFrame, cv::COLOR_BGR2HSV);
	std::vector<cv::Mat> channels;
	cv::split(HSVFrame, channels);
	cv::equalizeHist(channels[2], channels[2]);
	cv::merge(channels, HSVFrame);
	cv::imshow("Beacon Detector IMAGE ddddddd", HSVFrame);
}

void BeaconDetector::Morph(cv::Mat& frame)
{
	cv::Mat errodeElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
	cv::Mat dilateElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
	
	cv::erode(frame, frame, errodeElement);
	cv::erode(frame, frame, errodeElement);

	cv::dilate(frame, frame, dilateElement);
	cv::dilate(frame, frame, dilateElement);

}

cv::Vec3f BeaconDetector::ExtractHSV(cv::Rect pt)
{
	cv::Mat mask = HSVFrame.clone()(pt);
	mask.convertTo(mask, CV_32FC4);
	return mask.at<cv::Vec3f>(0, 0);
}

void BeaconDetector::ValidateBounds(cv::Vec3f& lower, cv::Vec3f& upper)
{
	cv::Vec3f lowerTemp, upperTemp;
	lowerTemp[0] = std::max(lower[0] - upper[0], 0.0f);
	lowerTemp[1] = std::max(lower[1] - upper[1], 0.0f);
	lowerTemp[2] = std::max(lower[2] - lower[2], 0.0f);

	upperTemp[0] = std::min(lower[0] + upper[0], 180.0f);
	upperTemp[1] = std::min(lower[1] + upper[1], 255.0f);
	upperTemp[2] = std::min(lower[2] + upper[2], 255.0f);
	lower = lowerTemp;
	upper = upperTemp;
}

cv::Mat BeaconDetector::ColorThreshold(cv::Mat frame, cv::Vec3f& lower, cv::Vec3f& upper)
{
	ValidateBounds(lower, upper);
	cv::Mat result;
	cv::inRange(frame, lower, upper, result);
	Morph(result);
	return result;
}

std::vector<cv::Rect> BeaconDetector::CannySegment(cv::Mat& threshold, int minArea, int maxArea)
{
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierahcy;
	cv::Mat output;
	cv::Canny(threshold, output, 100, 200, 3);
	cv::findContours(output, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
	std::vector<cv::Rect> potentials;
	cv::Mat display = rawFrame.clone();
	for (int i = 0; i < contours.size(); i++)
	{
		cv::Rect data = cv::boundingRect(cv::Mat(contours[i]));
		cv::rectangle(display, data, cv::Scalar(0, 0, 255), 1, 8, 0);
		cv::putText(display, std::to_string(data.area()), data.tl(), cv::FONT_HERSHEY_PLAIN, 0.6, cv::Scalar(255, 0, 255), 1, 8);
		if (data.x >= 0 && data.y >=  0 && data.x <=  rawFrame.size().width  && data.y <=  rawFrame.size().height)
		{
			if (data.area() >= std::min(minArea, MIN_AREA) && data.area() <= std::min(maxArea, MAX_AREA))
			{
				cv::rectangle(display, data, cv::Scalar(0, 255, 0), 1, 8, 0);
				potentials.push_back(data);
			}
		}
	}
	cv::imshow("Beacon Detector :: Canny results", display);
	return potentials;
}

cv::Rect BeaconDetector::Search(std::vector<cv::Rect>& data, BeaconData& previousReading)
{
	float dist = INT_MAX;
	cv::Rect result;
	cv::Mat searchOutput = rawFrame.clone();
	for (int i = 0; i < data.size(); i++)
	{
		float d = GetFitScore(data[i], previousReading);
		//std::cerr << d << std::endl;
		cv::putText(searchOutput, std::to_string(d), data[i].tl(), cv::FONT_HERSHEY_PLAIN, 0.6, cv::Scalar(0, 255, 255), 1, 8, false);
		if (d < dist)
		{
			result = data[i];
			dist = d;
		}
	}
	cv::imshow("Beacon Detector :: search results", searchOutput);
	return result;
}

cv::Vec3f BeaconDetector::Sample(cv::Rect window, cv::Vec3f& meanHSV)
{
	cv::Vec3f HSV = { 0.0f,0.0f,0.0f };
	std::vector<cv::Vec3f> inliers, result;
	int i = 0;
	int matches = 0;
	while (matches < SAMPLE_SIZE && i < MAX_ITER)
	{
		int x = rand() % ((window.x + window.width) - window.x + 1);
		int y = rand() % ((window.y + window.height) - window.y + 1);
		cv::Vec3f samplePt = ExtractHSV(cv::Rect(x, y, 1, 1));
		float result = HSVDiff(meanHSV, samplePt);
		if (result <= 0.1)
		{
			HSV[0] += samplePt[0];
			HSV[1] += samplePt[1];
			HSV[2] += samplePt[2];
			matches++;
		}
		i++;
	}

	if (matches > 0)
	{
		HSV[0] = HSV[0] / matches;
		HSV[1] = 0;//HSV[1] / matches;
		HSV[2] = 0;//HSV[2] / matches;
	}
	return HSV;
}

cv::Vec3f BeaconDetector::Sample(cv::Rect window)
{
	cv::Vec3f samplePt = ExtractHSV(cv::Rect(window.x + window.width * 0.5, window.y + window.height * 0.5, 1, 1));
	return Sample(window, samplePt);
}


void BeaconDetector::Detect(cv::Mat frame, std::vector<BeaconData> readings)
{
	counter++;
	beacons.clear();
	Convert(frame);
	for (int i = 0; i < readings.size(); i++)
		ScentedDetection(readings[i]);
	cv::Mat results = rawFrame.clone();
	for (int i = 0; i < beacons.size(); i++)
	{
		cv::rectangle(results, beacons[i].rect, cv::Scalar(beacons[i].HSV[0], 255,255), 1, 8, 0);
		cv::putText(results, beacons[i].ID, beacons[i].rect.tl(), cv::FONT_HERSHEY_PLAIN, 0.6, cv::Scalar(255, 255, 255), 1, 8);
	}
	cv::putText(results, std::to_string(counter), cv::Point(10, 20), cv::FONT_HERSHEY_PLAIN, 0.8, cv::Scalar(255, 255, 255), 1, 8);
	cv::imshow("Detection Becon", results);
}

void BeaconDetector::ScentedDetection(BeaconData& data)
{
	BeaconData temp(data);
	cv::Mat threshold = ColorThreshold(HSVFrame, data.HSV, data.range);
	cv::imshow("treshold", threshold);
	std::vector<cv::Rect> potentials = CannySegment(threshold, data.rect.area() * 0.5, data.rect.area() * 2);
	cv::Rect result;
	if (potentials.size() > 0)
	{
		result = Search(potentials, data);
		if (result.area() > 0)
		{
			temp.rect = result;
			//temp.HSV[0] += 15;
			//temp.HSV = Sample(temp.rect, data.HSV);
			temp.history.push_back(temp.HSV);
			beacons.push_back(temp);
		}
	}

}