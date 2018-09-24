#define STATE_SIZE 6
#define MEASUREMENT_SIZE 4
#define CONTROL_SIZE 0

//indexes for state matrices
#define X_POS  0
#define Y_POS  1
#define X_VEL  2
#define Y_VEL  3
#define WIDTH  4
#define HEIGHT 5

#include "Obstacle.h"
#include <math.h>
#include <iomanip>

using namespace std;

/// <summary>
/// Creates obstacle from point cluster
/// </summary>
Obstacle::Obstacle(vector<Point2f> position)
{
	expansionSearchWindow = 3;
	padding = 5;
	sizeSmoothing = 4;
	posSmoothing = 2;
	notFoundCount = 0;
	//search variables: These change how much a new reading can deviate from expectation before it will be discarded
	confidence = 0; //depending on confidence, object will be ignored or deleted

	//Kalman filter variables: These change the sensitivity of the filter to new readings
	locationNoise = 5e-2f;
	sizeNoise = 1e-1f;
	velocityNoise = 1e-4f;
	
	//Create Kalman filter
	CreateKalmanFilter(locationNoise, sizeNoise, velocityNoise);

	//Initialize Kalman filter state information from current state
	currentCluster = position;
	previousCluster = position;
	objectExpansion = getExpansionRate(currentCluster);
	correctedLocation = toRect(position);
	smoothedLocation = correctedLocation;

	positionHistory.push_back(correctedLocation);

	measurement = Mat::zeros(MEASUREMENT_SIZE, 1, CV_32F);
	measurement.at<float>(0) = correctedLocation.x + correctedLocation.width / 2;
	measurement.at<float>(1) = correctedLocation.y + correctedLocation.height / 2;
	measurement.at<float>(2) = correctedLocation.width;
	measurement.at<float>(3) = correctedLocation.height;

	predictedState = Mat(STATE_SIZE, 1, CV_32F);
	predictedState.at<float>(X_POS) = measurement.at<float>(0);
	predictedState.at<float>(Y_POS) = measurement.at<float>(1);
	predictedState.at<float>(X_VEL) = 0;
	predictedState.at<float>(Y_VEL) = 0;
	predictedState.at<float>(WIDTH) = measurement.at<float>(2);
	predictedState.at<float>(HEIGHT) = measurement.at<float>(3);

	correctedState = myFilter.statePre = myFilter.statePost = predictedState;
}

/// <summary>
/// Creates obstacle from rectangle
/// </summary>
Obstacle::Obstacle(Rect position)
{
	expansionSearchWindow = 3;
	padding = 5;
	sizeSmoothing = 4;
	posSmoothing = 2;
	notFoundCount = 0;
	//search variables: These change how much a new reading can deviate from expectation before it will be discarded
	confidence = 0; //depending on confidence, object will be ignored or deleted

					//Kalman filter variables: These change the sensitivity of the filter to new readings
	locationNoise = 5e-2f;
	sizeNoise = 1e-1f;
	velocityNoise = 1e-4f;

	//Create Kalman filter
	CreateKalmanFilter(locationNoise, sizeNoise, velocityNoise);

	//Initialize Kalman filter state information from current state
	//currentCluster = 0;
	//previousCluster = position;
	//objectExpansion = getExpansionRate(currentCluster);
	correctedLocation = position;
	smoothedLocation = correctedLocation;

	positionHistory.push_back(correctedLocation);

	measurement = Mat::zeros(MEASUREMENT_SIZE, 1, CV_32F);
	measurement.at<float>(0) = correctedLocation.x + correctedLocation.width / 2;
	measurement.at<float>(1) = correctedLocation.y + correctedLocation.height / 2;
	measurement.at<float>(2) = correctedLocation.width;
	measurement.at<float>(3) = correctedLocation.height;

	predictedState = Mat(STATE_SIZE, 1, CV_32F);
	predictedState.at<float>(X_POS) = measurement.at<float>(0);
	predictedState.at<float>(Y_POS) = measurement.at<float>(1);
	predictedState.at<float>(X_VEL) = 0;
	predictedState.at<float>(Y_VEL) = 0;
	predictedState.at<float>(WIDTH) = measurement.at<float>(2);
	predictedState.at<float>(HEIGHT) = measurement.at<float>(3);

	correctedState = myFilter.statePre = myFilter.statePost = predictedState;
}

/// <summary>
/// Creates Kalman filter for tracking
/// </summary>
void Obstacle::CreateKalmanFilter(float locationNoise, float sizeNoise, float velocityNoise)
{
	//create Kalman filter of correct size
	myFilter = KalmanFilter(STATE_SIZE, MEASUREMENT_SIZE, CONTROL_SIZE, CV_32F);

	//intialize transition matrix as identity matrix
	setIdentity(myFilter.transitionMatrix);

	//intiailize measurement matrix, this is the weighting applied to each reading, set to 1
	myFilter.measurementMatrix = Mat::zeros(MEASUREMENT_SIZE, STATE_SIZE, CV_32F);
	myFilter.measurementMatrix.at<float>(0) = 1.0f;					    // [ 1 0 0 0 0 0 ]
	myFilter.measurementMatrix.at<float>(7) = 1.0f;					    // [ 0 1 0 0 0 0 ]
	myFilter.measurementMatrix.at<float>(16) = 1.0f;					// [ 0 0 0 0 1 0 ]
	myFilter.measurementMatrix.at<float>(23) = 1.0f;					// [ 0 0 0 0 0 1 ]

	// Process Noise Covariance Matrix (this is the ongoing noise we expect in our modelling
	myFilter.processNoiseCov.at<float>(0) = locationNoise;						// [ x   0   0    0    0    0 ]
	myFilter.processNoiseCov.at<float>(7) = locationNoise;						// [ 0   y   0    0    0    0 ]
	myFilter.processNoiseCov.at<float>(14) = velocityNoise;						// [ 0   0   vx   0    0    0 ]
	myFilter.processNoiseCov.at<float>(21) = velocityNoise;						// [ 0   0   0    vy   0    0 ]
	myFilter.processNoiseCov.at<float>(28) = sizeNoise;							// [ 0   0   0    0    h    0 ]
	myFilter.processNoiseCov.at<float>(35) = sizeNoise;							// [ 0   0   0    0    0    w ]

	// Measurent Noise Covariance Matrix (this is the noise we expect ot be present in our first reading)
	myFilter.measurementNoiseCov.at<float>(0) = locationNoise;						// [ Ex   0   0     0]
	myFilter.measurementNoiseCov.at<float>(5) = locationNoise;						// [ 0    Ey  0     0]
	myFilter.measurementNoiseCov.at<float>(10) = sizeNoise;							// [ 0    0   Ew    0]
	myFilter.measurementNoiseCov.at<float>(15) = sizeNoise;							// [ 0    0   0    Eh]

	//this is the error present in our readings, since it is our first reading, error can be assumed to be zero
	//however this must be non zero value for Kalman filter to properly update postion, set to 1
	myFilter.errorCovPre.at<float>(0) = 1; //px
	myFilter.errorCovPre.at<float>(7) = 1; //px
	myFilter.errorCovPre.at<float>(14) = 1;
	myFilter.errorCovPre.at<float>(21) = 1;
	myFilter.errorCovPre.at<float>(28) = 1; //pxA
	myFilter.errorCovPre.at<float>(35) = 1; //px
}

/// <summary>
/// Read Kalmlan filter in order to generate new position estimates
/// </summary>
void Obstacle::ReadKalmanFilter()
{
	//update filter with time delta
	float dT = .1; //change in time from previous reading, currently using .1 as placeholder, doesn't matter for video feeds
	myFilter.transitionMatrix.at<float>(2) = dT; 
	myFilter.transitionMatrix.at<float>(9) = dT; 

	//call predict to update filters state prediction
	myFilter.predict();

	//clone predicted state from filter (cloning to preserve info)
	predictedState = myFilter.statePre.clone(); 

	//use predicted state to update predictec location
	predictedLocation.width = predictedState.at<float>(WIDTH);
	predictedLocation.height = predictedState.at<float>(HEIGHT);
	predictedLocation.x = predictedState.at<float>(X_POS) - predictedLocation.width / 2;
	predictedLocation.y = predictedState.at<float>(Y_POS) - predictedLocation.height / 2;
}

float Obstacle::getUrgency()
{
	return objectExpansion;
}

/// <summary>
/// Update Kalman filter using predicted state
/// </summary>
void Obstacle::UpdateKalmanFilter()
{
	measurement.at<float>(0) = predictedState.at<float>(X_POS);
	measurement.at<float>(1) = predictedState.at<float>(Y_POS);
	measurement.at<float>(2) = predictedState.at<float>(WIDTH);
	measurement.at<float>(3) = predictedState.at<float>(HEIGHT);

	notFoundCount += 1;

	//correct filter based on measurement
	myFilter.correct(measurement);

	//clone statepost to correctedState (cloning to preserve information)
	correctedState = myFilter.statePost.clone();

	//update corrected location from corrected state
	correctedLocation.width = correctedState.at<float>(WIDTH);
	correctedLocation.height = correctedState.at<float>(HEIGHT);
	correctedLocation.x = correctedState.at<float>(X_POS) - predictedLocation.width / 2;
	correctedLocation.y = correctedState.at<float>(Y_POS) - predictedLocation.height / 2;

	//increases corrected location size for safety
	PadLocation(correctedLocation); 

	//Save location and centrepoint to history
	positionHistory.push_back(correctedLocation);
	centrePointHistory.push_back(Point(correctedState.at<float>(X_POS), correctedState.at<float>(Y_POS))); //add point to obstacles history

	//get smoothed location
	smoothedLocation = SmoothedRectangle(positionHistory, posSmoothing, sizeSmoothing);

	//update cluster
	previousCluster = currentCluster;
	calcObjectExpansion();
}

/// <summary>
/// update Kalman fitler from singel reading
/// </summary>
void Obstacle::UpdateKalmanFilter(vector<Point2f> & reading)
{
	//clear not found coun and increment object confidence
	notFoundCount = 0;
	confidence++;

	//temp rectangle to store reading
	Rect temp = toRect(reading);

	//update measurement matric from reading
	measurement.at<float>(0) = temp.x + temp.width / 2;
	measurement.at<float>(1) = temp.y + temp.height / 2;
	measurement.at<float>(2) = temp.width;
	measurement.at<float>(3) = temp.height;

	//correct filter based on measurement
	myFilter.correct(measurement);

	//clone statepost to correctedState (cloning to preserve information)
	correctedState = myFilter.statePost.clone();

	//update corrected location from corrected state
	correctedLocation.width = correctedState.at<float>(WIDTH);
	correctedLocation.height = correctedState.at<float>(HEIGHT);
	correctedLocation.x = correctedState.at<float>(X_POS) - predictedLocation.width / 2;
	correctedLocation.y = correctedState.at<float>(Y_POS) - predictedLocation.height / 2;

	//increases corrected location size for safety
	PadLocation(correctedLocation); 

	//Save location and centrepoint to history
	positionHistory.push_back(correctedLocation);
	centrePointHistory.push_back(Point(correctedState.at<float>(X_POS), correctedState.at<float>(Y_POS))); //add point to obstacles history

	//get smoothed location
	smoothedLocation = SmoothedRectangle(positionHistory, posSmoothing, sizeSmoothing);

	//update cluster info on obstacle
	previousCluster = currentCluster;
	currentCluster = reading;
	calcObjectExpansion();
}

/// <summary>
/// update kalman filter from multiple readings
/// </summary>
void Obstacle::UpdateKalmanFilter(vector<vector<Point2f>>& readings)
{
	vector<Point2f> points;//new vector to hold merged results

	//combine readings
	for (int i = 0; i < readings.size(); i++)
	{
		points.insert(points.begin(), readings[i].begin(), readings[i].end());
	}

	//update kalman fitler from merged result
	UpdateKalmanFilter(points);
}

/// <summary>
/// calucalte rate of expansion of objects
/// </summary>
void Obstacle::calcObjectExpansion()
{
	float localHistory = 0;
	int count = 0;
	int window = 0;
	if (expHist.size() > expansionSearchWindow)
		window = expHist.size() - expansionSearchWindow;
	std::cerr << "PRINT OBJECT HISTORY" << std::endl;
	for (int i = window; i < expHist.size(); i++)
	{
		localHistory += expHist[i];
		std::cerr << expHist[i] << std::endl;
		count++;
	}
	if (count == 0)
		localHistory = 1;
	else
		localHistory = localHistory / count;

	// update history 
	float currentExpansion = getExpansionRate(currentCluster);
	float previousExpansion = getExpansionRate(previousCluster);

	if (currentExpansion != previousExpansion)
	{
		expHist.push_back(1 / (previousExpansion / currentExpansion));

	}
	else
		expHist.push_back(1);

	std::cerr << "Local history\t" << localHistory << std::endl;
	std::cerr << "current Expansion" << currentExpansion << std::endl;
	objectExpansion = (localHistory / currentExpansion);
	objectExpansion = 1 / objectExpansion;
	std::cerr << "OBJECT EXPASNION \t" << objectExpansion << std::endl;
}

/// <summary>
/// return objects rate of expansion
/// </summary>
float Obstacle::getExpansionRate(const vector<Point2f>& in)
{
	if (in.size() > 0)
	{
		Vec4f data;
		fitLine(in, data, CV_DIST_L2, 0, 0.01, 0.01);
		c.x = data[2]; // denotes center (FOE)
		c.y = data[3];
		float d = 0.0;
		for (int i = 0; i < in.size(); i++)
		{
			d += sqrt(((in[i].x - c.x) * (in[i].x - c.x) + (in[i].y - c.y) * (in[i].y - c.y)));
		}
		d = d / in.size();
		return d;
	}
	else
		return 1; // should never be reached; but failsafe. 
}

/// <summary>
/// Returns a smoothed rectangle, based on the given size and position smoothing values
/// </summary>
Rect Obstacle::SmoothedRectangle(vector<Rect> rects, int positionWindowSize, int sizeWindowSize)
{
	//limit smoothin windows if they are larger than the given vector
	if (sizeWindowSize > rects.size() || sizeWindowSize == 0)
		sizeWindowSize = rects.size();

	if (positionWindowSize > rects.size() || positionWindowSize == 0)
		positionWindowSize = rects.size();


	//sum position and sizes
	float sumX = 0;
	float sumY = 0;
	float sumWidth = 0;
	float sumHeight = 0;

	for (int i = rects.size() - positionWindowSize; i < rects.size(); i++)
	{
		sumX += rects[i].x;
		sumY += rects[i].y;
	}

	for (int i = rects.size() - sizeWindowSize; i < rects.size(); i++)
	{
		sumWidth += rects[i].width;
		sumHeight += rects[i].height;
	}

	//calcualte averages
	int avX = sumX / positionWindowSize;
	int avY = sumY / positionWindowSize;
	int avWidth = sumWidth / sizeWindowSize;
	int avHeight = sumHeight / sizeWindowSize;

	//return soothed rectangle
	return Rect (avX, avY, avWidth, avHeight);
}


/// <summary>
/// enlarge rectangle based on padding value
/// </summary>
void Obstacle::PadLocation(Rect & toPad)
{
	toPad.x -= padding / 2;
	toPad.y -= padding / 2;
	toPad.width += padding;
	toPad.height += padding;
}


/// <summary>
/// draw obstacle onto frame
/// </summary>
void Obstacle::Draw(Mat & frame)
{
	//get current and predicted center from state information
	Point center, futureCenter;
	center.x = correctedState.at<float>(X_POS);
	center.y = correctedState.at<float>(Y_POS);
	futureCenter.x = correctedState.at<float>(X_POS) + correctedState.at<float>(X_VEL);
	futureCenter.y = correctedState.at<float>(Y_POS) + correctedState.at<float>(Y_VEL);
	c = center;

	//draw object and futurepoint line onto cloned frame
	Mat frameCopy = frame.clone(); //clone input frame
	rectangle(frameCopy, smoothedLocation, CV_RGB(0, 255, 0), -1); //draw rectangle onto clone
	line(frameCopy, center, futureCenter, CV_RGB(0, 0, 255), 1, 8, 0); //draw a line from current to predicted center
	addWeighted(frame, .5, frameCopy, .5, 0, frame); //merge cloned frame with original (this is to create transperancy)
}

Obstacle::~Obstacle()
{
}

//FUNCTIONS BELOW THIS POINT ARE TO FACILITATE DEBUGGING/REPORTING, AND SHOULD NOT FUNDAMENTALLY EFFECT OPERATION

void Obstacle::ClearHistory()
{
	centrePointHistory.clear(); //history is vector of historical centrpoints
}

int Obstacle::GetConfidence()
{
	return confidence;
}

void Obstacle::DrawHistory(Mat frame)
{
	int count = 100;
	//for each centrePoiint in obstacle history (other than the first)
	for (int i = centrePointHistory.size() - 1; i > 0; i--)
	{
		line(frame, centrePointHistory[i], centrePointHistory[i - 1], CV_RGB(0, 255, 0), 1, 8, 0); //draw a line to the previous history point
		count--;
		if (count == 0)
			break;
	}
}


Rect Obstacle::toRect(const std::vector<cv::Point2f>& data)
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

void Obstacle::DrawCluster(Mat & frame)
{
	for (int i = 0; i < currentCluster.size(); i++)
	{
		circle(frame, currentCluster[i], 1, CV_RGB(0, 0, 0), 1, 8, 0);
	}

	std::ostringstream strs, str2;
	strs << objectExpansion;
	std::string str = strs.str();
	putText(frame, str, c,FONT_HERSHEY_COMPLEX_SMALL, 0.5, cvScalar(230, 230, 255), 1, CV_AA);
	
	
}