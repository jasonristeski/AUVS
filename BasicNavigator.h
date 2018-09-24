/*	------------------------------------
BASICNAVIGATOR <--- NAVIGATOR
------------------------------------
Description:
Navigator child. Used by caller for calculating navigational information and controlling
Comms data inputs/outputs. 
------------------------------------
Fields:
_communicator (Comms)*
_initialSetupComplete (Bool)*
_commandDirections (Dirs)*
_vehicleDimensions (XYSize[3])
Functions:
public void InitialSetup()*
public float GetPollValue()*
public bool DecideDirections(vector<NavObject>, vector<NavObject>)
public bool DebugDecideDirections(*Mat, vector<NavObject>, vector<NavObject>)
public bool SendDirectionCommands()*

* = inherited
------------------------------------*/
/*
#pragma once
#include "XYSize.h"
#include "Navigator.h"

//stores region points (top-left and bottom-right positions)
struct RegionPoints
{
	int x0 = 0;
	int x1 = 0;
	int y0 = 0;
	int y1 = 0;
};

class BasicNavigator : public Navigator
{
private:
	int _screenWidth;
	int _screenHeight;

	//heat map (or "threat" map). Uses small values to indicate threat level at specific region of map, fish-bot will attempt to move so that the region with lowest/no threat will be centered
	std::vector<std::vector<short>> _map;

	//holds vehicle lengths for each side
	XYSize _vehicleDimensions[3];

	//map regions
	RegionPoints _centerRegion;

	//emergency behavious timer variables
	double _lastTime;
	double _timer;
	double _maxIdleTime;


	BasicNavigator(){ }
public:
	BasicNavigator(int ScreenWidth, int ScreenHeight, float FrontHeight, float FrontWidth, float TopHeight, float TopWidth, float SideHeight, float SideWidth)
	{
		_screenWidth = ScreenWidth;
		_screenHeight = ScreenHeight;

		if (_screenWidth < 320)
			_screenWidth = 320;
		if (_screenHeight < 240)
			_screenHeight = 240;

		_vehicleDimensions[0].Height = FrontHeight;
		_vehicleDimensions[0].Width = FrontWidth;
		_vehicleDimensions[1].Height = TopHeight;
		_vehicleDimensions[1].Width = TopWidth;
		_vehicleDimensions[2].Height = SideHeight;
		_vehicleDimensions[2].Width = SideWidth;

		_lastTime = -1;
		_timer = 0;
		_maxIdleTime = 5.0;
	}

	//if initital setup complete flag is false, sets up and sets flag to true
	void InitialSetup() override;

	//calculates and returns polling value (may query data from external source for calculations)
	float GetPollValue() override;

	//takes in object data, which is then used to calculate the best direction to move in order to avoid collisions, and which direction flags should be used via external components
	//sets direction sent flag to false on start
	bool DecideDirections(std::vector<NavObject> BeaconData, std::vector<NavObject> InputData) override;
	bool DebugDecideDirections(cv::Mat* currentFrame, std::vector<NavObject> BeaconData, std::vector<NavObject> InputData) override;


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
};*/