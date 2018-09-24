#pragma once
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include <iostream>

using namespace cv;
using namespace std;

class Obstacle
{
public:
	Obstacle(vector<Point2f> position);
	Obstacle(Rect position);
	~Obstacle();
	void ClearHistory();
	void DrawHistory(Mat frame);
	int GetConfidence();
	void DrawCluster(Mat & frame);
	void Draw(Mat & frame);
	void UpdateKalmanFilter(vector<Point2f> & reading);
	void UpdateKalmanFilter(vector<vector<Point2f>> & reading);
	void UpdateKalmanFilter();
	void ReadKalmanFilter();
	float getUrgency();
	int notFoundCount;
	bool updated;
	Rect correctedLocation, predictedLocation, smoothedLocation;
	vector<vector<Point2f>> newReadings;
	vector<Point2f> currentCluster, previousCluster;
	vector<Point> centrePointHistory;
private:
	int confidence;
	int sizeSmoothing, posSmoothing;
	float objectExpansion;
	bool found;
	float locationNoise, sizeNoise, velocityNoise;
	int padding;
	int expansionSearchWindow;
	Mat predictedState, correctedState, measurement;
	KalmanFilter myFilter;
	Point2f c;
	vector<float> expHist;
	vector<Rect> positionHistory;


	Rect SmoothedRectangle(vector<Rect> rects, int positionWindowSize, int sizeWinsowSize);
	float getExpansionRate(const vector<Point2f>& in);
	void PadLocation(Rect & toPad);
	void calcObjectExpansion();
	void CreateKalmanFilter(float locationNoise, float sizeNoise, float velocityNoise);
	Rect toRect(const std::vector<cv::Point2f>& data);
};

