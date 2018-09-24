#pragma once
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include <iostream>
#include <math.h>
#include "Tracker.h"
#include "Obstacle.h"
#include "Detector.h"
#include "NavObject.h"

class Tracker
{
public:
	Tracker(Detector* objectDetector, string trackerType);
	~Tracker();

	void SetShowDisplay();
	void ToggleTracing();
	void ToggleDetectionOverlay();
	void ToggleDebug();
	void ToggleDetectionWindows();
	vector<NavObject> myNavObjects;
	void RefreshAll();
	void ClearTrace();
	void PrintResults();


	bool tracing, detectionOverlay, debug, showDisplay;
	
private:
	void UpdateObstacles(vector<vector<Point2f>>& readings);
	cv::Rect ToRect(const std::vector<cv::Point2f>& data);
	vector<NavObject> CreateNavObjects(vector<Obstacle> obs);
	void MatchReadings(vector<vector<Point2f>>& data, vector<Obstacle>& objects);
	void CreateObstacles(vector<vector<Point2f>>& readings);
	float sizeDiff(vector<Point2f> data, Obstacle object);
	float posDiff(vector<Point2f> data, Obstacle object);
	float pointDiff(vector<Point2f> data, Obstacle object);
	float GetFitScore(vector<Point2f> data, Obstacle object, float a, float b, float c);
	vector<vector<Point2f>>  ReadDetector();

	Detector* obstacleDetector;
	vector<Obstacle> detectedObstacles;
	vector<vector<Point2f>> readings;
	vector<Rect> historicalReadings, detectionOverlayRects;
	int confidenceThreshold, smallestRect, notFoundThreshold, scoreThreshold;

	string trackerType;
};

