#pragma once
// cv 
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/features2d.hpp"
// std
#include <vector>

struct EuclideanPredicate
{
	int d;
	EuclideanPredicate(int distance) : d(distance * distance) {}

	bool operator()(const cv::Point2f& a, const cv::Point2f& b) const
	{
		return ((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y)) < d;
	}

};


class Detector
{
public:
	Detector();
	Detector(std::string path);
	Detector(int device);
	~Detector();


	std::vector<std::vector<cv::Point2f>> GetMovingObjects();
	std::vector<std::vector<cv::Point2f>> GetDataArray(std::string dataName);
	std::vector<std::vector<cv::Point2f>> GetStaticObjects();
	std::vector<std::vector<cv::Point2f>> GetUFO();
	cv::Mat getRawFrame() const;
	cv::VideoCapture capture;

	void Detect();
	// Flags 
	void ToggleDebug();
	void toggleDisplay();


private:

	void Init();

	void Read(cv::Mat& frame);
	float CalcDistance(const cv::Point2f & pt1, const cv::Point2f & pt2);
	void Convert(cv::Mat& frame);


	static void Cluster(std::vector<cv::Point2f>& dataPts, std::vector<std::vector<cv::Point2f>>& out, int frequencyThreshold, int lowPassThreshold, int highPassThreshold);

	void PointCluster(); // threaded version
	void DisplayCluster(); // for debugging

	void TrackFlow(std::vector<cv::Point2f>& prev, int pass);
	void filterPoints(std::vector<cv::KeyPoint>& in, std::vector<cv::Point2f>& out, int size);
	void match(std::vector<cv::Point2f>& in, int distance);
	void DisplayOutput();

	bool debug;
	bool output;

	cv::Ptr<cv::Feature2D> fast;
	cv::Mat previousFrame, currentFrame, rawFrame;

	std::vector<cv::Point2f> movingPts, staticPts, unknownPts;
	std::vector<std::vector<cv::Point2f>> movingObjects, staticObjects, unknownObjects;
};


