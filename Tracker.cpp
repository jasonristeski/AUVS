//#include "stdafx.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include "NavObject.h"
#include <iostream>
#include <math.h>
#include "Tracker.h"
#include "Obstacle.h"
#include "Detector.h"
#include "BeaconDetector.h"

using namespace cv;
using namespace std;

/// <summary>
/// Called by constructors to initialize Tracker
/// </summary>
Tracker::Tracker(Detector* detectorObject, string dataSource)
{
	trackerType = dataSource;
	obstacleDetector = detectorObject;

	confidenceThreshold = 10; //obstacles will only be drawn with a confidence level above this value
	smallestRect = 100; //the smallest reading which will be considered as an obstacle 
	notFoundThreshold = 5; //number of empty readings before an object is considered to be lost
	scoreThreshold = 300; // maximum allowed difference for a reading to be matched to an obstacle
	
	showDisplay = false;
	tracing = false;
	detectionOverlay = false;
	debug = false;
}

/// <summary>
/// Refreshes tracker, from getting new readings through to creating new obstacles
/// </summary>
void Tracker::RefreshAll()
{
	readings = ReadDetector();
	UpdateObstacles(readings); //use new readings to update existing obstacles
	CreateObstacles(readings); //create new obstacles from remaining readings
	PrintResults(); //print results to screen
	myNavObjects = CreateNavObjects(detectedObstacles); //Create navigation objects from obstacles

}

/// <summary>
/// Gets cluster data from detector
/// </summary>
vector<vector<Point2f>> Tracker::ReadDetector()
{
	vector<vector<Point2f>> clusters; //temporary vector to hold filtered results
	vector <vector<Point2f>> dataVector = obstacleDetector->GetDataArray(trackerType); 
	
	
	for (int i = 0; i < dataVector.size(); i++) //for each reading
	{
		Rect temp = ToRect(dataVector[i]); //create temp rectangle from cluster
		if (temp.width*temp.height > smallestRect) //check that rectangle is of sufficient size
		{
			clusters.push_back(dataVector[i]); //push cluster to clusters
		}
	}

	return clusters; //return vector containing results of acceptable size
}

/// <summary>
/// Updates all tracked obstacles based on new readings
/// </summary>
void Tracker::UpdateObstacles(vector<vector<Point2f>> & readings)
{
	for (int i = 0; i < detectedObstacles.size(); i++) //for each trackec obstacle
	{
		//if obstacles has nto been found for several readings, delet from vector
		if (detectedObstacles[i].notFoundCount > notFoundThreshold)
		{
			detectedObstacles.erase(detectedObstacles.begin() + i);
			i--;
		}
		//otherwise have the obstacle read it's kalman filter, then update with new readings
		else
		{
			detectedObstacles[i].ReadKalmanFilter();
		}
	}

	//match new readings to existing obstacles
	MatchReadings(readings, detectedObstacles);
}

/// <summary>
///Creates new obstacles from unused readings
/// </summary>
void Tracker::CreateObstacles(vector<vector<Point2f>> & readings)
{
	//for each remaingin reading
	for (int i = 0; i < readings.size(); i++)
	{
		Obstacle newObstacle = Obstacle(readings[i]); //make new obstacle
		detectedObstacles.push_back(newObstacle); //push obstacle to vector
	}
}

/// <summary>
/// if show display is enabled, prints tracking display
/// </summary>
void Tracker::PrintResults()
{
	if (showDisplay) //if showDisplay is enabled
	{
		Mat currentFrame = obstacleDetector->getRawFrame().clone(); //clone current frame from detector
		for (int i = 0; i <detectedObstacles.size(); i++) //for each detected obstacle
		{
			//only draw obstacle if it is of acceptable confidence
			if (detectedObstacles[i].GetConfidence() > confidenceThreshold) 
			{
				detectedObstacles[i].Draw(currentFrame); //draw object to image
				if (tracing)
					detectedObstacles[i].DrawHistory(currentFrame); //trace histroy (if enabled)

					detectedObstacles[i].DrawCluster(currentFrame);
			}
		}

		//draw detection overlay boxes (if enabled)
		if (detectionOverlay)
		{
			int count = 30;
			for (int i = historicalReadings.size() - 1; i >= 0; i--)
			{
				Mat frameCopy = currentFrame.clone();
				rectangle(currentFrame, historicalReadings[i], CV_RGB(0, 0, 255), -1);
				addWeighted(currentFrame, .1, frameCopy, .9, 0, currentFrame);
				count--;
				if (count == 0)
					break;
			}
		}

		//create and update display window
		namedWindow(string(trackerType), WINDOW_NORMAL);
		imshow(string(trackerType), currentFrame);

		//if system is at appropriate frame
		//print image
		int frameCount = obstacleDetector->capture.get(CV_CAP_PROP_POS_FRAMES);
		if (frameCount % 3 == 0 && frameCount > 635)
		{
			string filename = trackerType + "/" + to_string(frameCount) + "_GT.png";// "_groundruth.png";
			vector<int> compression_params;
			compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
			compression_params.push_back(9);
			imwrite(filename, currentFrame, compression_params);
		}
		//imwrite("Test", currentFrame);
	}
}

/// <summary>
/// Create a vector of navigation objects from currently tracked obstacles
/// </summary>
vector<NavObject> Tracker::CreateNavObjects(vector<Obstacle> obs)
{
	int xMin, yMin, xMax, yMax;
	myNavObjects.clear(); //clear old values

						  //for each tracked obstacle
	for (int i = 0; i < obs.size(); i++)
	{
		//if obstacle is of sufficient confidence
		if (obs[i].GetConfidence() > confidenceThreshold)
		{
			//ensure that min and max coordinates do not exceed frame
			xMin = obs[i].smoothedLocation.x;
			if (xMin < 0)
				xMin = 0;

			yMin = obs[i].smoothedLocation.y;
			if (yMin < 0)
				yMin = 0;

			xMax = obs[i].smoothedLocation.x + obs[i].smoothedLocation.width;
			if (xMax > 319)
				xMax = 319;

			yMax = obs[i].smoothedLocation.y + obs[i].smoothedLocation.height;
			if (yMax > 239)
				yMax = 239;

			//create navigation obstacle from coordinates
			NavObject temp(xMin, yMin, xMax - xMin, yMax - yMin, obs[i].getUrgency(), 1);
			//add object to navigation obejct vector
			myNavObjects.push_back(temp);
		}
	}
	return myNavObjects;
}

/// <summary>
/// Match tracked obstacles ot new readings
/// </summary>
void Tracker::MatchReadings(vector<vector<Point2f>> & data, vector<Obstacle> & objects)
{
	//set updated status on each obstacel to false
	for (int i = 0; i < objects.size(); i++)
		objects[i].updated = false;

	//for each reading
	for (int i = 0; i < data.size(); i++)
	{
		float bestScore = INT_MAX;
		float thisScore = INT_MAX;
		bool found = false;
		int closestIndex = 0;

		//for each obstacle
		for (int j = 0; j < objects.size(); j++)
		{
			//get matching score between reading and osbtacle
			thisScore = GetFitScore(data[i], objects[j], .1, 1, 0);

			//save index of best match
			if (thisScore < bestScore)
			{
				bestScore = thisScore;
				closestIndex = j;
			}
		}

		//if best match is good enough
		if (bestScore < scoreThreshold)
		{
			//update new readings on obstacle
			detectedObstacles[closestIndex].newReadings.push_back(data[i]);
			//remove reading from vector
			data.erase(data.begin() + i);
			i--;
		}
	}

	//for each object, update depending on number of readings that have been found
	for (int i = 0; i < objects.size(); i++)
	{
		//update obstacle using it's prediction
		if (objects[i].newReadings.size() == 0)
		{
			objects[i].UpdateKalmanFilter();
		}

		//update obstacle from single reading
		if (objects[i].newReadings.size() == 1)
		{
			objects[i].UpdateKalmanFilter(objects[i].newReadings[0]);
		}

		//update obstacle from combination of multiple readings
		if (objects[i].newReadings.size() > 1)
		{
			cout << "UPDATING OBSTACLE USING " << objects[i].newReadings.size() << " READINGS" << endl;
			objects[i].UpdateKalmanFilter(objects[i].newReadings);
		}

		//clear readin vector on obstacle
		objects[i].newReadings.clear();
	}
}

/// <summary>
/// return the difference in size between two rectangles
/// </summary>
float Tracker::sizeDiff(vector<Point2f> data, Obstacle object)
{
	Rect temp = ToRect(data);
	float heightDiff = abs(temp.height - object.predictedLocation.height);
	float widthDiff = abs(temp.width - object.predictedLocation.width);
	float sizeDiff = heightDiff * widthDiff;
	return sizeDiff;
}

/// <summary>
/// return the difference in location between two rectangels
/// </summary>
float Tracker::posDiff(vector<Point2f> data, Obstacle object)
{
	Rect temp = ToRect(data);
	Point center1 = Point(temp.x + temp.width / 2, temp.y + temp.height / 2);
	Point center2 = Point(object.predictedLocation.x + object.predictedLocation.width / 2, object.predictedLocation.y + object.predictedLocation.height / 2);
	
	float distance = sqrt(pow(center1.x - center2.x, 2) + pow(center1.y - center2.y, 2));
	return distance;
}

/// <summary>
/// return the difference innumber of poitns between two rectangles
/// </summary>
float Tracker::pointDiff(vector<Point2f> data, Obstacle object)
{
	int pointDifference = data.size() - object.currentCluster.size();
	return abs(pointDifference);
}

/// <summary>
/// calcualte a fit score for two rectangles, as a weighted sum of size, postiion, and point difference
/// </summary>
float Tracker::GetFitScore(vector<Point2f> data, Obstacle object, float a, float b, float c)
{
	float sizeDifference = sizeDiff(data, object);
	float posDifference = posDiff(data, object);
	float pointDifference = pointDiff(data, object);

	float score = sizeDifference * a + posDifference * b + pointDifference * c;

	return score;
}

/// <summary>
/// convert point cluster to rectangle
/// </summary>
cv::Rect Tracker::ToRect(const std::vector<cv::Point2f>& data)
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

Tracker::~Tracker()
{
}

//FUNCTIONS BELOW THIS POINT ARE TO FACILITATE DEBUGGING/REPORTING, AND SHOULD NOT FUNDAMENTALLY EFFECT OPERATION

void Tracker::ClearTrace()
{
	historicalReadings.clear(); //clear historical detection readings
	for (int i = 0; i < detectedObstacles.size(); i++) //for each obstacle
	{
		detectedObstacles[i].ClearHistory(); //clear history
	}
}

void Tracker::SetShowDisplay()
{
	showDisplay = true;
}

void Tracker::ToggleTracing()
{
	tracing = !tracing; //toggle tracing
	for (int i = 0; i < detectedObstacles.size(); i++) //for each obstacle
	{
		detectedObstacles[i].ClearHistory(); //clear history
	}
}

void Tracker::ToggleDetectionOverlay()
{
	detectionOverlay = !detectionOverlay; //toggle deteciton overlay
	historicalReadings.clear(); //cear historical readings
}

void Tracker::ToggleDebug()
{
	debug = !debug; //toggle debug
}

void Tracker::ToggleDetectionWindows()
{
	obstacleDetector->toggleDisplay();
}


