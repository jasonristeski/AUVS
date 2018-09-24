/*	------------------------------------
	ATTRACT REPULSE NAV
	------------------------------------
	Description:
		Navigator class. Used by caller for calculating navigational information and controlling
		Comms data inputs/outputs.
	------------------------------------
	Fields:
		screenWidth (int)
		screenHeight (int)

		halfWidth (int)
		halfHeight (int)
		maxDistance (int)

		directions (Directions)

		map (cv::Mat)

		turnActive (bool0
		lastTurnTime (double)
		turnTimer (double)
		turnMaxIdleTime (double)

		lastTime (double)
		timer (double)
		maxIdleTime (double)

	Functions:
		public void InitialSetup()
		public float GetPollValue()
		public bool DecideDirections(vector<NavObject>, vector<NavObject>)
		public bool DebugDecideDirections(*Mat, vector<NavObject>, vector<NavObject>)
		public bool RegionPointsSet(RegionPoints reg)
		public RegionPoints SetRegionPoints(int X0, int X1, int Y0, int Y1)

------------------------------------*/

#pragma once
#include "XYSize.h"
#include "Directions.h"
#include "NavObject.h"

// cv 
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/features2d.hpp"

//stores region points (top-left and bottom-right positions)
struct RegionPoints
{
	int x0 = 0;
	int x1 = 0;
	int y0 = 0;
	int y1 = 0;
};

class AttractRepulseNav
{
private:
	int screenWidth;
	int screenHeight;

	int halfWidth;
	int halfHeight;
	int maxDistance;

	Directions directions;

	//used to visually convery data and/or nav results
	cv::Mat map;

	//turn behaviour variables
	bool turnActive;
	double lastTurnTime;
	double turnTimer;
	double turnMaxIdleTime;

	//emergency behaviour timer variables
	double lastTime;
	double timer;
	double maxIdleTime;
        
        bool _initialSetupComplete;


	AttractRepulseNav(){ }
public:
	AttractRepulseNav(int ScreenWidth, int ScreenHeight, float FrontHeight, float FrontWidth, float TopHeight, float TopWidth, float SideHeight, float SideWidth)
	{
		screenWidth = ScreenWidth;
		screenHeight = ScreenHeight;

		if (screenWidth < 320)
			screenWidth = 320;
		if (screenHeight < 240)
			screenHeight = 240;

		directions = Directions();

		lastTime = -1;
		timer = 0;
		maxIdleTime = 5.0;

		turnActive = false;
		lastTurnTime = -1;
		turnTimer = 0;
		turnMaxIdleTime = 10.0;
	}

	//if initital setup complete flag is false, sets up and sets flag to true
	void InitialSetup();

	//calculates and returns polling value (may query data from external source for calculations)
	float GetPollValue();

	//takes in object data, which is then used to calculate the best direction to move in order to avoid collisions, and which direction flags should be used via external components
	//sets direction sent flag to false on start
	bool DecideDirections(std::vector<NavObject> BeaconData, std::vector<NavObject> InputData);
	bool DebugDecideDirections(cv::Mat* currentFrame, std::vector<NavObject> BeaconData, std::vector<NavObject> InputData);


	bool RegionPointsSet(RegionPoints reg)
	{
		if (reg.x0 == 0 && reg.x1 == 0 && reg.y0 == 0 && reg.y1 == 0)
			return false;
		else
			return true;
	}

	RegionPoints SetRegionPoints(int X0, int X1, int Y0, int Y1)
	{
		RegionPoints result;
		result.x0 = X0;
		result.x1 = X1;
		result.y0 = Y0;
		result.y1 = Y1;

		return result;
	}
};