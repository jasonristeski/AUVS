#pragma once
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include <iostream>
#include <math.h>
#include "Tracker.h"
#include "Obstacle.h"
#include "Detector.h"
#include "NavObject.h"

class BeaconTracker
{
public:
	BeaconTracker(Detector* objectDetector, string trackerType);
	~BeaconTracker();

	void SetShowDisplay();
	void ToggleTracing();
	void ToggleDetectionOverlay();
	void ToggleDebug();
	void ToggleDetectionWindows();
	void RefreshAll();
	void ClearTrace();
	void PrintResults();

	vector<NavObject> beaconNavObjects;
	bool beaconTracing, beaconDetectionOverlay, beaconDebug, beaconShowDisplay;

private:
	void UpdateObstacles(vector<Rect>& readings);
	cv::Rect toRect(const std::vector<cv::Point2f>& data);
	vector<NavObject> CreateNavObjects(vector<Obstacle> obs);
	void MatchReadings(vector<Rect>& data, vector<Obstacle>& objects);
	void CreateObstacles(vector<Rect>& readings);
	float sizeDiff(Rect data, Obstacle object);
	float posDiff(Rect data, Obstacle object);
	float GetFitScore(Rect data, Obstacle object, float a, float b);
	vector<Rect>  ReadDetector();

	Detector* myDetector;
	vector<Obstacle> detectedObstacles;
	vector<Rect> readings;
	vector<Rect> historicalReadings, detectionOverlayRects;
	int confidenceThreshold, smallestRect, notFoundThreshold, scoreThreshold;

	string trackerType;
};

