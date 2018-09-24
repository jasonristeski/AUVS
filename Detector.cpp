#include "Detector.h"


//cv
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/imgproc.hpp"


//std
//#include <thread>
#include <iostream>
#include <numeric>   
#include <map>
#include <time.h>
#include <fstream>
using namespace cv;
using namespace std;



// Gloabl params for tuning 

// Point filter threshold
int pointQuality = 4;


// Optical Flow Params
int pyramid = 3;
int iter = 20;
float quality = 0.2f;
int winSize = 21;

//matching threshold 
float movingPred = 21;
float staticPred = 30;

// Iterative static Point cluster
int staticFreqThreshold = 15; // min cluster size
int staticLowPass = 18; // low pass
int staticHighPass = 25; // high pass

						 // Iterative moving point cluster
int movingFreqThreshold = 5; // min cluster size
int movingLowPass = 16; // low pass
int movingHighPass = 25; // high pass




cv::Rect toRect(const std::vector<cv::Point2f>& data)
{
	cv::Point2f max, min;
	max.x = INT_MIN;
	max.y = INT_MIN;

	min.x = INT_MAX;
	min.y = INT_MAX;

	for (int i = 0; i < data.size(); i++)
	{
		if (data[i].x > max.x)
			max.x = data[i].x;

		if (data[i].y > max.y)
			max.y = data[i].y;

		if (data[i].x < min.x)
			min.x = data[i].x;

		if (data[i].y < min.y)
			min.y = data[i].y;
	}
	cv::Rect r(min.x, min.y, max.x - min.x, max.y - min.y);
	return r;
}


Detector::Detector()
{

}

/// <summary>
/// Constructor - Accepts path of video playback 
/// </summary>
/// <param name="path">Path for video</param>
Detector::Detector(std::string path)
{

	capture.open(path);
	Init();
}


/// <summary>
/// Constructor - Accepts device ID for capturing frames  
/// </summary>
/// <param name="device">Device ID to use as camera (0 is default in CV)</param>
Detector::Detector(int device)
{
	capture.open(device);
	Init();
}


/// <summary>
/// Destrcutor - Destructs 
/// </summary>
Detector::~Detector()
{

}

/// <summary>
/// Called by constructors to initialize Detector 
/// </summary>
void Detector::Init()
{
	fast = cv::FastFeatureDetector::create(2, true, 2);
	debug = false;
	output = false;
	//writer.open("perf.csv");

}

/// <summary>
/// Toggles debug flag for output
/// </summary>
void Detector::ToggleDebug()
{
	debug = !debug;
}

/// <summary>
/// Toggles display flag for output
/// </summary>
void Detector::toggleDisplay()
{
	output = !output;
}

/// <summary>
/// Returns raw frame from capture for ohter modules to use
/// </summary>
/// <returns>Raw Frame from capture</returns>
cv::Mat Detector::getRawFrame() const
{
	return rawFrame;
}

/// <summary>
/// Returns vector of moving point cluster 
/// </summary>
/// <returns>Moving Objects</returns>
std::vector<std::vector<cv::Point2f>> Detector::GetMovingObjects()
{
	return movingObjects;
}

/// <summary>
/// Returns vector of moving point cluster 
/// </summary>
/// <returns>Moving Objects</returns>
std::vector<std::vector<cv::Point2f>> Detector::GetDataArray(string dataName)
{
	if (dataName == "static")
	{
		/*int frameCount = capture.get(CV_CAP_PROP_POS_FRAMES);
		//std::cout << "                 FRAME: " << frameCount << std::endl;
		int modulus = frameCount % 45;
		if (modulus == 0)
		{
			std::cout << "                 FRAME: " << frameCount << std::endl;

			std::string filename = "SD/" + to_string(frameCount) + "_SD.png";// "_groundruth.png";
			cv::Mat result = cv::Mat::zeros(cv::Size(320, 240), CV_8UC3);
			for (int i = 0; i < staticObjects.size(); i++)
			{
				cv::rectangle(result, toRect(staticObjects[i]), cv::Scalar(0, 255, 0), -1, 8, 0);
			}
			vector<int> compression_params;
			compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
			compression_params.push_back(9);
			imwrite(filename, result, compression_params);
		}*/
		return staticObjects;
	}
	if (dataName == "dynamic")
	{
		/*int frameCount = capture.get(CV_CAP_PROP_POS_FRAMES);
		//std::cout << "                 FRAME: " << frameCount << std::endl;
		int modulus = frameCount % 45;
		if (modulus == 0)
		{
			std::cout << "                 FRAME: " << frameCount << std::endl;

			std::string filename = "DD/" + to_string(frameCount) + "_DD.png";// "_groundruth.png";
			cv::Mat result = cv::Mat::zeros(cv::Size(320, 240), CV_8UC3);
			for (int i = 0; i < movingObjects.size(); i++)
			{
				cv::rectangle(result, toRect(movingObjects[i]), cv::Scalar(0, 0, 255), -1, 8, 0);
			}
			vector<int> compression_params;
			compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
			compression_params.push_back(9);
			imwrite(filename, result, compression_params);
		}*/
		return movingObjects;
	}
	else
	{
		vector<std::vector<cv::Point2f>> temp;
		return temp;
	}
}

/// <summary>
/// Returns vector of static point cluster 
/// </summary>
/// <returns>Static Objects</returns>
std::vector<std::vector<cv::Point2f>> Detector::GetStaticObjects()
{
	return staticObjects;
}

/// <summary>
/// Returns vector of unclassified point cluster 
/// These points have not moved significantly enough to be classified as dynamic; but too much to be static 
/// </summary>
/// <returns>Unknown Points </returns>
std::vector<std::vector<cv::Point2f>> Detector::GetUFO()
{
	return unknownObjects;
}

/// <summary>
/// Converts frame to 320x240 and performs a median Blur
/// </summary>
/// <param name="frame">Frame (passed by ref)</param>
void Detector::Convert(cv::Mat& frame)
{

	cv::resize(frame, frame, cv::Size(320, 240));
	if (false)
	{
		int w = frame.size().width * 0.85;
		int h = frame.size().height * 0.85;
		cv::Rect roi((frame.size().width - w) * 0.5, (frame.size().height - h) * 0.5, w, h);
		frame = frame(roi);
	}

	frame.copyTo(rawFrame);
	int frameCount = capture.get(CV_CAP_PROP_POS_FRAMES);
	/*if (frameCount % 3 == 0)
	{
		string filename = "GT/" + to_string(frameCount) + "_GT.png";// "_groundruth.png";
		vector<int> compression_params;
		compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
		compression_params.push_back(9);
		imwrite(filename, frame, compression_params);
	}*/
	cv::medianBlur(frame, frame, 3);
}

/// <summary>
/// Reads from capture device and stores in frame 
/// </summary>
/// <param name="frame">Frame (passed by ref.)</param>
void Detector::Read(cv::Mat& frame)
{
	capture.read(frame);
	if (frame.empty())
	{
		std::cerr << "End of video stream. Press any key to close" << std::endl;
		std::cin.ignore();
		exit(0);
	}
	else
		Convert(frame);
}

/// <summary>
/// Caclulates Euclidean Distance 
/// </summary>
/// <param name="pt1">Point A</param>
/// <param name="pt2"> Point B</param>
/// <returns>Euclidean Distance </returns>
float Detector::CalcDistance(const cv::Point2f& pt1, const cv::Point2f& pt2)
{
	return sqrt(((pt1.x - pt2.x) * (pt1.x - pt2.x)) + ((pt1.y - pt2.y) * (pt1.y - pt2.y)));
}

/// <summary>
/// When associated flag is called, displays output points detected.
/// </summary>
void Detector::DisplayOutput()
{
	cv::Mat drawMovingPts = cv::Mat::zeros(rawFrame.size(), rawFrame.type());
	cv::Mat drawStaticPts = cv::Mat::zeros(rawFrame.size(), rawFrame.type());
	cv::Mat drawUnknownPts = cv::Mat::zeros(rawFrame.size(), rawFrame.type());

	for (int i = 0; i < (movingPts.size() + staticPts.size() + unknownPts.size()); i++)
	{
		if (i < movingPts.size())
			cv::circle(drawMovingPts, movingPts[i], 1, cv::Scalar(255, 255, 255), -1);
		if (i < staticPts.size())
			cv::circle(drawStaticPts, staticPts[i], 1, cv::Scalar(255, 255, 255), -1);
		if (i < unknownPts.size())
			cv::circle(drawUnknownPts, unknownPts[i], 1, cv::Scalar(255, 255, 255), -1);
	}
	cv::putText(rawFrame, std::to_string(capture.get(CV_CAP_PROP_POS_FRAMES)), cv::Point(10, 20), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1, 8, false);
	cv::imshow("Detector::Input", rawFrame);
	cv::imshow("Detector::Moving Points", drawMovingPts);
	cv::imshow("Detector::Static Points", drawStaticPts);
	cv::imshow("Detector::Unknown Points", drawUnknownPts);


}

/// <summary>
/// Performs nearest neighbour clustering passed off High/Low pass frequency Thresholds:
/// Low pass clustering is performed, where clusters below a minimum point frequency are discarded
/// High pass frequency then joins all the remaining clusters together to form the object point cluster. 
/// </summary>
/// <param name="dataPts">Input raw point vector </param>
/// <param name="out">Output clustered point object vector </param>
/// <param name="frequencyThreshold">Frequency Threshold for point count</param>
/// <param name="lowPassThreshold">Low pass nearest neighbour threshold</param>
/// <param name="highPassThreshold">High pass nearest neighbour threshold</param>
void Detector::Cluster(std::vector<cv::Point2f>& dataPts, std::vector<std::vector<cv::Point2f>>& out, int frequencyThreshold, int lowPassThreshold, int highPassThreshold)
{
	if (dataPts.size() > 0)
	{
		// Low pass clustering 
		std::vector<int> labels;
		int count = cv::partition(dataPts, labels, EuclideanPredicate(lowPassThreshold));

		// Point frequency filter
		std::map<int, int> freq;
		for (int i = 0; i < dataPts.size(); i++)
			freq[labels[i]]++; // for each potential cluster, increment associated map index


		std::vector<std::vector<cv::Point2f>> tempVector(count);
		for (int i = 0; i < dataPts.size(); i++)
		{
			if (freq[labels[i]] > frequencyThreshold)
				tempVector[labels[i]].push_back(dataPts[i]);
		}

		// Reconstructs raw point vector based off 
		std::vector<cv::Point2f> temp(count);
		for (int i = 0; i < dataPts.size(); i++)
		{
			if (freq[labels[i]] > frequencyThreshold)
				temp.push_back(dataPts[i]);
		}

		dataPts = temp;
		labels.clear();
		count = cv::partition(dataPts, labels, EuclideanPredicate(highPassThreshold));
		out.resize(count);
		for (int i = 0; i < dataPts.size(); i++)
		{
			out[labels[i]].push_back(dataPts[i]);
		}
	}
}

/// <summary>
/// Performs cluster on each vector 
/// </summary>
void Detector::PointCluster()
{
	Cluster(staticPts, staticObjects, staticFreqThreshold, staticLowPass, staticHighPass);
	Cluster(movingPts, movingObjects, movingFreqThreshold, movingLowPass, movingHighPass);
	Cluster(unknownPts, unknownObjects, movingFreqThreshold, movingLowPass, movingHighPass);

}

/// <summary>
/// When approriate flag is set, displays clusters to output
/// </summary>
void Detector::DisplayCluster()
{
	/*Cluster(staticPts, staticObjects, staticFreqThreshold, staticLowPass, staticHighPass);
	Cluster(movingPts, movingObjects, movingFreqThreshold, movingLowPass, movingHighPass);
	Cluster(unknownPts, unknownObjects, movingFreqThreshold, movingLowPass, movingHighPass);
	*/
	cv::Mat drawing = cv::Mat::zeros(rawFrame.size(), rawFrame.type());
	cv::Mat removed = cv::Mat::zeros(rawFrame.size(), rawFrame.type());
	if (staticPts.size() > 0)
	{

		std::vector<int> labels;
		int count = cv::partition(staticPts, labels, EuclideanPredicate(staticLowPass));
		std::map<int, int> freq;
		for (int i = 0; i < staticPts.size(); i++)
			freq[labels[i]]++;
		std::vector<cv::Point2f> temp(count);
		for (int i = 0; i < staticPts.size(); i++)
		{
			if (freq[labels[i]] > staticFreqThreshold)
				temp.push_back(staticPts[i]);
		}
		staticPts = temp;
		labels.clear();
		count = cv::partition(staticPts, labels, EuclideanPredicate(staticHighPass));
		staticObjects.resize(count);
		for (int i = 0; i < staticPts.size(); i++)
		{
			staticObjects[labels[i]].push_back(staticPts[i]);
		}

		for (int i = 0; i < staticObjects.size(); i++)
		{
			for (int j = 0; j < staticObjects[i].size(); j++)
			{
				cv::line(drawing, staticObjects[i][0], staticObjects[i][j], cv::Scalar(0, 255, 0), 1, 8, 0);
			}
		}
	}

	if (movingPts.size() > 0)
	{
		std::vector<int> labels;
		int count = cv::partition(movingPts, labels, EuclideanPredicate(movingLowPass));
		std::map<int, int> freq;
		for (int i = 0; i < movingPts.size(); i++)
			freq[labels[i]]++;

		std::vector<cv::Point2f>temp(count);
		for (int i = 0; i < movingPts.size(); i++)
		{
			if (freq[labels[i]] > movingFreqThreshold)
				temp.push_back(movingPts[i]);
		}

		movingPts = temp;
		labels.clear();
		count = cv::partition(movingPts, labels, EuclideanPredicate(movingHighPass));
		movingObjects.resize(count);
		for (int i = 0; i < movingPts.size(); i++)
			movingObjects[labels[i]].push_back(movingPts[i]);

		for (int i = 0; i < movingObjects.size(); i++)
		{
			for (int j = 0; j < movingObjects[i].size(); j++)
			{
				cv::line(drawing, movingObjects[i][0], movingObjects[i][j], cv::Scalar(0, 0, 255), 1, 8, 0);
			}
		}
	}

	if (unknownPts.size() > 0)
	{
		std::vector<int> labels;
		int count = cv::partition(unknownPts, labels, EuclideanPredicate(11));
		std::map<int, int> freq;
		for (int i = 0; i < unknownPts.size(); i++)
			freq[labels[i]]++;
		std::vector<cv::Point2f> temp(count);
		for (int i = 0; i < unknownPts.size(); i++)
		{
			if (freq[labels[i]] > 5)
				temp.push_back(unknownPts[i]);
		}

		unknownPts = temp;
		labels.clear();
		count = cv::partition(unknownPts, labels, EuclideanPredicate(21));
		unknownObjects.resize(count);
		for (int i = 0; i < unknownPts.size(); i++)
		{
			unknownObjects[labels[i]].push_back(unknownPts[i]);
		}

		for (int i = 0; i < unknownObjects.size(); i++)
		{
			for (int j = 0; j < unknownObjects[i].size(); j++)
			{
				cv::line(drawing, unknownObjects[i][0], unknownObjects[i][j], cv::Scalar(255, 0, 255), 1, 8, 0);
			}
		}
	}
	cv::imshow("Detector::Clustering22", drawing);

}


/// <summary>
/// Performs matching against unknown points based off
/// </summary>
/// <param name="in"> Vector to match against </param>
/// <param name="distance"> Distance Predicate</param>
void Detector::match(std::vector<cv::Point2f>& in, int distance)
{
	if (unknownPts.size() > 0)
	{
		for (int i = 0; i < in.size(); i++)
		{
			int idx = 0;
			for (int j = 0; j < unknownPts.size(); j++)
			{
				float dist = sqrt(((in[i].x - unknownPts[j - idx].x) * (in[i].x - unknownPts[j - idx].x)) + ((in[i].y - unknownPts[j - idx].y) * (in[i].y - unknownPts[j - idx].y)));
				if (dist < distance)
				{
					in.push_back(unknownPts[j - idx]);
					unknownPts.erase(unknownPts.begin() + j - idx);
					idx++;
				}
			}
		}
	}
}
/// <summary>
/// Based off a temporal smootheness constraint, a point must be stable over N frames to be deemed significant
/// Uses Lucas Kanade Optical flow to track points
/// </summary>
/// <param name="prev"> Previous point vector </param>
/// <param name="pass"> Recursive termination counter  </param>
void Detector::TrackFlow(std::vector<cv::Point2f>& prev, int pass)
{

	// Optical flow paramaters 
	std::vector<uchar> status; // status = 0 if object lost, 1 if found
	std::vector<float> error;
	std::vector<cv::Point2f> next;

	// experiment with Guassian Blur 
	//cv::GaussianBlur(currentFrame, currentFrame, cv::Size(5, 5), 1, 0, cv::BORDER_DEFAULT);
	cv::calcOpticalFlowPyrLK(previousFrame, currentFrame, prev, next, status, error, cv::Size(winSize, winSize), pyramid, cv::TermCriteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS, iter, quality));
	pass--;
	if (pass != 0)
	{
		// Keep tracking points
		cv::swap(previousFrame, currentFrame);
		Read(currentFrame);
		TrackFlow(next, pass);
	}
	else
	{
		/*// remove bad points
		int idx = 0;
		for (int i = 0; i < status.size(); i++)
		{
		// Point was not tracked for N frames or has gone outside the bounds of the current frame
		if ((status.at(i) == 0) || (next.at(i - idx).x < 0) || (next.at(i - idx).y < 0))
		{
		next.erase(next.begin() + (i - idx));
		prev.erase(prev.begin() + (i - idx));
		idx++;
		} */


		if (!next.empty())
		{
			std::vector<uchar> inliersFwd; // To be used to find dominant m

			cv::Mat H = cv::findHomography(prev, next, CV_RANSAC, 2.0, inliersFwd, 1000, 0.95); // Estimate homography matrix fwd

			if (!H.empty())
			{
				std::vector<cv::Point2f> warpedPts(next);
				cv::perspectiveTransform(prev, next, H);
				for (int i = 0; i < next.size(); i++)
				{
					float d = sqrt(((next[i].x - warpedPts[i].x)*(next[i].x - warpedPts[i].x)) + ((next[i].y - warpedPts[i].y)*(next[i].y - warpedPts[i].y)));
					if (d >= 1)
						movingPts.push_back(next[i]);
					else if (d < 0.5)
						staticPts.push_back(next[i]);
					else
						unknownPts.push_back(next[i]);

				}
			}
			else
			{
				for (int i = 0; i < next.size(); i++)
				{
					float d = sqrt(((next[i].x - prev[i].x)*(next[i].x - prev[i].x)) + ((next[i].y - prev[i].y)*(next[i].y - prev[i].y)));
					if (d > 1)
						movingPts.push_back(next[i]);
					else if (d < 0.5)
						staticPts.push_back(next[i]);
					else
						unknownPts.push_back(next[i]);
				}
			}
		}
		match(movingPts, movingPred);
		match(staticPts, staticPred);
	}
}


/// <summary>
/// Filters points based off their response (qaulity) 
/// </summary>
/// <param name="in"> Keypoints </param>
/// <param name="out">Point 2f </param>
/// <param name="size">Max vector size</param>
void Detector::filterPoints(std::vector<cv::KeyPoint>& in, std::vector<cv::Point2f>& out, int size)
{
	if (in.size() > 0)
	{
		//Sorts points based on their quaility 
		std::sort(in.begin(), in.end(), [](const cv::KeyPoint& a, const cv::KeyPoint& b)
		{
			return a.response > b.response;
		});



		if (in.size() < size)
			size = in.size();
		for (int i = 0; i < size; i++)
		{
			// terminating clause if vector could not be filled; ensures a minimum level of quality is established
			// rather than taking degraded bad points 
			if (in.at(i).response < pointQuality)
				break;
			else
				out.push_back(in.at(i).pt);
		}

	}
}

void Detector::Detect()
{

	staticPts.clear();
	movingPts.clear();
	unknownPts.clear();
	movingObjects.clear();
	staticObjects.clear();
	unknownObjects.clear();


	std::vector<cv::Point2f> prevPts;
	std::vector<cv::KeyPoint> pts;

	Read(previousFrame);
	Read(currentFrame);
	// for final release std::thread beaconThread(&Detector::detectArtificalObjects,this);
	//detectArtificalObjects();
	fast->detect(previousFrame, pts);
	filterPoints(pts, prevPts, 750);
	if (prevPts.size() > 0)
	{
		TrackFlow(prevPts, 2);
		//PointCluster();
		//DisplayCluster(); // for debugging
		
		
		if (output)
		{
			DisplayCluster();
		}
		else if (debug)
		{
			DisplayCluster();
			DisplayOutput();
		}
			
		else
			PointCluster();
			
	}
}

