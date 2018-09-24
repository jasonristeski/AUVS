/*#pragma once
#include "BasicNavigator.h"
#include <atltime.h>
#include <iostream>
#include<sstream>
#include <math.h>

using namespace std;
using namespace cv;

void BasicNavigator::InitialSetup()
{
	//make sure map is clear
	_map.clear();

	//set up map size
	for (int i = 0; i < _screenWidth; i++)
	{
		//temporary variable for creating 2d map vector row
		vector<short> temp;

		for (int j = 0; j < _screenHeight; j++)
		{
			temp.push_back(0);
		}

		_map.push_back(temp);
	}

	//get thirds (for preparing centre region)
	int widthThird = ceil(_screenWidth / 3);
	int heightThird = ceil(_screenHeight / 3);

	//calculate centre region
	_centerRegion.x0 = widthThird;
	_centerRegion.y0 = heightThird;
	_centerRegion.x1 = _screenWidth - widthThird;
	_centerRegion.y1 = _screenHeight - heightThird;

	cout << "map width is " << _map.size() << "\n";
	cout << "map height is " << _map[0].size() << "\n\n";

	_initialSetupComplete = true;
}

float BasicNavigator::GetPollValue()
{
	//NOT IMPLEMENTED - returns 0.2

	//call comms, get speed
	//using speed, calculate next polling value and return it (min. x, max. y)
	float result = (float)0.2;
	return result;
}

bool BasicNavigator::DecideDirections(vector<NavObject> BeaconData, vector<NavObject> InputData)
{
	//check initialized (critical error avoidance)
	if (!_initialSetupComplete)
	{
		cout << "NAV: Not initialized\n";
		return false;
	}

	//first, reset map to blank (no threats)
	for (int i = 0; i < _map.size(); i++)
	{
		for (int j = 0; j < _map[i].size(); j++)
		{
			_map[i][j] = 0;
		}
	}

	//fill map with threat values (thread value = urgancy (based on rate of expansion)
	//REMINDER: threat values are not permenant. 0 = no threat, 1-39 means insignificant threat, 40+ means threat worth avoiding (higher number = bigger threat), 100 means critical threat (too late, no possible way to avoid collision)
	for (int i = 0; i < InputData.size(); i++)
	{
		for (int y = InputData[i].GetY(); y <= InputData[i].GetY() + InputData[i].GetLength(); y++)
		{
			for (int x = InputData[i].GetX(); x <= InputData[i].GetX() + InputData[i].GetWidth(); x++)
			{
				if (_map[x][y] < InputData[i].GetUrgency())
					_map[x][y] = InputData[i].GetUrgency();
			}
		}
	}

	//check center of screen for safe passage
	RegionPoints bestSectionRegion;
	bool centreSafePassage = false;

	RegionPoints safeRegion = _centerRegion;

	//iterators used for scanning
	int x = _centerRegion.x0;
	int y = _centerRegion.y0;

	vector<RegionPoints> safeLineSegments;

	//first step, check for start/stop points per row of map
	bool firstMapScanComplete = false;
	bool firstPointFound = false;

	while (!firstMapScanComplete)
	{
		//check map pos for threat, if no worthwhile threat...
		if (_map[x][y] <= 39)
		{
			//if x is at final pixel for the row, or not currently logging a line, start logging a line
			if ((x == _centerRegion.x0 || _map[x - 1][y] > 39) && !firstPointFound)
			{
				firstPointFound = true;
				RegionPoints temp;
				temp.x0 = x;
				temp.y0 = temp.y1 = y;
				safeLineSegments.push_back(temp);
			}
		}
		//otherwise if worthwhile threat, complete the log
		else if (_map[x][y] > 39 && firstPointFound)
		{
			safeLineSegments.back().x1 = x - 1;
			firstPointFound = false;
		}

		// if the current pixel was the last in the row, end any current logs and continue to next row (or end if already on last row)
		if (x == _centerRegion.x1)
		{
			if (firstPointFound)
			{
				safeLineSegments.back().x1 = x;
				firstPointFound = false;
			}

			if (y == _centerRegion.y1)
				firstMapScanComplete = true;
			else
			{
				x = _centerRegion.x0;
				y++;
			}
		}
		//otherwise, increment x
		else
		{
			x++;
		}
	}

	//second step, go through each line and compare with each other for safe zones
	int yCount = 0;
	for (int i = 0; i < safeLineSegments.size(); i++)
	{
		//reset yCount
		yCount = 0;
		bool failedCheck = false;

		//for each safe zone entry, check if it's for the next line. if it isn't, end checking, otherwise check it's validity and increase yCount if acceptable
		for (int j = i + 1; j < safeLineSegments.size() && !failedCheck; j++)
		{

			if (safeLineSegments[j].y0 == (safeLineSegments[i].y0 + 1 + yCount))
			{
				if (safeLineSegments[i].x0 >= safeLineSegments[j].x0 && safeLineSegments[i].x1 <= safeLineSegments[j].x1)
					yCount++;
			}
			else
				failedCheck = true;
		}

		//if no best region set, automatically becomes best region
		if (!RegionPointsSet(bestSectionRegion))
			bestSectionRegion = SetRegionPoints(safeLineSegments[i].x0, safeLineSegments[i].x1, safeLineSegments[i].y0, safeLineSegments[i].y1 + yCount);
		else
		{
			//if region is bigger than currently stored best region (contains more pixels), it becomes new best region
			if (((safeLineSegments[i].x1 - safeLineSegments[i].x0) * (yCount + 1)) > ((bestSectionRegion.x1 - bestSectionRegion.x0) * (bestSectionRegion.y1 - bestSectionRegion.y0)))
				bestSectionRegion = SetRegionPoints(safeLineSegments[i].x0, safeLineSegments[i].x1, safeLineSegments[i].y0, (safeLineSegments[i].y1 + yCount));
		}
	}

	//third step, test best region to see if it's large enough for the fishbot (for now, must be at least 1/3 the size)
	if (bestSectionRegion.x1 - bestSectionRegion.x0 >= ceil((_centerRegion.x1 - _centerRegion.x0) / 2) && bestSectionRegion.y1 - bestSectionRegion.y0 >= ceil((_centerRegion.y1 - _centerRegion.y0) / 2))
	{
		centreSafePassage = true;
		int centrePointX = _centerRegion.x0 + (ceil(_centerRegion.x1 - _centerRegion.x0) / 3);
		int centrePointY = _centerRegion.y0 + (ceil(_centerRegion.y1 - _centerRegion.y0) / 3);

		//if safe passage is central, move forward, and adjust direction to head for exact centre of safe passage
		if (centrePointX > bestSectionRegion.x0 && centrePointX < bestSectionRegion.x1 && centrePointY > bestSectionRegion.y0 && centrePointY < bestSectionRegion.y1)
		{
			_commandDirections.SetUpDown(0);
			_commandDirections.SetForBack(1);
			_commandDirections.SetLeftRight(0);

			int bestSectionRegionCentreX = (ceil((bestSectionRegion.x1 - bestSectionRegion.x0) / 2) + bestSectionRegion.x0);
			int bestSectionRegionCentreY = (ceil((bestSectionRegion.y1 - bestSectionRegion.y0) / 2) + bestSectionRegion.y0);

			if (centrePointX > bestSectionRegionCentreX)
			{
				if (centrePointX - bestSectionRegionCentreX > 20)
					_commandDirections.SetLeftRight(-1);
			}
			else if (centrePointX < bestSectionRegionCentreX)
			{
				if (bestSectionRegionCentreX - centrePointX > 20)
					_commandDirections.SetLeftRight(1);
			}

			if (centrePointY > bestSectionRegionCentreY)
			{
				if (centrePointY - bestSectionRegionCentreY > 20)
					_commandDirections.SetUpDown(-1);
			}
			else if (centrePointY < bestSectionRegionCentreY)
			{
				if (bestSectionRegionCentreY - centrePointY > 20)
					_commandDirections.SetUpDown(1);
			}
		}
		//otherwise, adjust so heading towards the found safe passage
		else
		{
			if (centrePointX > bestSectionRegion.x1)
				_commandDirections.SetLeftRight(-1);
			else if (centrePointX < bestSectionRegion.x0)
				_commandDirections.SetLeftRight(1);
			else
				_commandDirections.SetLeftRight(0);

			if (centrePointY > bestSectionRegion.y1)
				_commandDirections.SetUpDown(-1);
			else if(centrePointY < bestSectionRegion.y0)
				_commandDirections.SetUpDown(1);
			else
				_commandDirections.SetUpDown(0);
			
			_commandDirections.SetForBack(1);
		}
	}

	//if safe passage center, send commands and return true
	if (centreSafePassage)
	{	
		SendDirectionCommands();
		return true;
	}

	//fourth step, if no safe passage center, check whole screen to find safest passage

	//reset/recycle variables where appropriate
	bestSectionRegion = RegionPoints();
	bool safePassage = false;

	safeRegion = RegionPoints();

	RegionPoints fullMap;
	fullMap.x0 = 0;
	fullMap.y0 = 0;
	fullMap.x1 = _screenWidth - 1;
	fullMap.y1 = _screenHeight - 1;

	//iterators used for scanning
	x = fullMap.x0;
	y = fullMap.y0;

	safeLineSegments.clear();

	//fifth step, check for start/stop points per row of full map
	firstMapScanComplete = false;
	firstPointFound = false;

	while (!firstMapScanComplete)
	{
		if (_map[x][y] <= 39)
		{
			if ((x == fullMap.x0 || _map[x - 1][y] > 39) && !firstPointFound)
			{
				firstPointFound = true;
				RegionPoints temp;
				temp.x0 = x;
				temp.y0 = temp.y1 = y;
				safeLineSegments.push_back(temp);
			}
		}
		else if (_map[x][y] > 39 && firstPointFound)
		{
			safeLineSegments.back().x1 = x - 1;
			firstPointFound = false;
		}

		if (x == fullMap.x1)
		{
			if (firstPointFound)
			{
				safeLineSegments.back().x1 = x;
				firstPointFound = false;
			}

			if (y == fullMap.y1)
				firstMapScanComplete = true;
			else
			{
				x = fullMap.x0;
				y++;
			}
		}
		else
		{
			x++;
		}
	}

	//sixth step, go through each line and compare with each other for safe zones
	yCount = 0;
	for (int i = 0; i < safeLineSegments.size(); i++)
	{
		//reset yCount
		yCount = 0;
		bool failedCheck = false;

		//for each safe zone entry, check if it's for the next line. if it isn't, end checking, otherwise check it's validity and increase yCount if acceptable
		for (int j = i + 1; j < safeLineSegments.size() && !failedCheck; j++)
		{

			if (safeLineSegments[j].y0 == (safeLineSegments[i].y0 + 1 + yCount))
			{
				if (safeLineSegments[i].x0 >= safeLineSegments[j].x0 && safeLineSegments[i].x1 <= safeLineSegments[j].x1)
					yCount++;
			}
			else
				failedCheck = true;
		}

		//if no best region set, automatically becomes best region
		if (!RegionPointsSet(bestSectionRegion))
			bestSectionRegion = SetRegionPoints(safeLineSegments[i].x0, safeLineSegments[i].x1, safeLineSegments[i].y0, safeLineSegments[i].y1 + yCount);
		else
		{
			//if region is bigger than currently stored best region (contains more pixels), it becomes new best region
			if (((safeLineSegments[i].x1 - safeLineSegments[i].x0) * (yCount + 1)) > ((bestSectionRegion.x1 - bestSectionRegion.x0) * (bestSectionRegion.y1 - bestSectionRegion.y0)))
				bestSectionRegion = SetRegionPoints(safeLineSegments[i].x0, safeLineSegments[i].x1, safeLineSegments[i].y0, (safeLineSegments[i].y1 + yCount));
		}
	}

	//seventh step, test best region to see if it's large enough for the fishbot (for now, must be at least 1/3 the size)
	if (bestSectionRegion.x1 - bestSectionRegion.x0 >= ceil((_centerRegion.x1 - _centerRegion.x0) / 2) && bestSectionRegion.y1 - bestSectionRegion.y0 >= ceil((_centerRegion.y1 - _centerRegion.y0) / 2))
	{
		safePassage = true;
		int centrePointX = _centerRegion.x0 + (ceil(_centerRegion.x1 - _centerRegion.x0) / 3);
		int centrePointY = _centerRegion.y0 + (ceil(_centerRegion.y1 - _centerRegion.y0) / 3);

		_commandDirections.SetUpDown(0);
		_commandDirections.SetForBack(1);
		_commandDirections.SetLeftRight(0);

		if (bestSectionRegion.x1 < centrePointX)
			_commandDirections.SetLeftRight(-1);
		else if (bestSectionRegion.x0 > centrePointX)
			_commandDirections.SetLeftRight(1);
		if (bestSectionRegion.y1 < centrePointY)
			_commandDirections.SetUpDown(-1);
		else if (bestSectionRegion.y0 > centrePointY)
			_commandDirections.SetUpDown(1);
	}

	//if no safe passage, proceed with emergency actions
	if (!safePassage)
	{
		//if no start time, set start time
		if (_lastTime < 0)
		{
			LARGE_INTEGER li;
			QueryPerformanceCounter(&li);

			_lastTime = (li.QuadPart / 1000.0);
		}
		else
		{
			double lDeltaTime = 0;
			LARGE_INTEGER li;
			QueryPerformanceCounter(&li);

			lDeltaTime = (li.QuadPart / 1000.0) - _lastTime;
			_lastTime = (li.QuadPart / 1000.0);

			_timer += lDeltaTime;
		}

		//if less than 5 seconds, stay still
		if (_timer < _maxIdleTime)
		{
			_commandDirections.SetUpDown(0);
			_commandDirections.SetForBack(0);
			_commandDirections.SetLeftRight(0);
		}
		//otherwise, rotate left
		else
		{
		_commandDirections.SetUpDown(0);
		_commandDirections.SetForBack(0);
		_commandDirections.SetLeftRight(-1);
		}
	}
	else if (_timer > 0)
	{
		_timer = 0;
		_lastTime = -1;
	}

	//send direction commands, then return safePassage value
	SendDirectionCommands();

	return safePassage;
}

bool BasicNavigator::DebugDecideDirections(Mat* currentFrame, vector<NavObject> BeaconData, vector<NavObject> InputData)
{
	//check initialized (critical error avoidance)
	if (!_initialSetupComplete)
	{
		cout << "NAV: Not initialized\n";
		return false;
	}

	//first, reset map to blank (no threats)
	for (int i = 0; i < _map.size(); i++)
	{
		for (int j = 0; j < _map[i].size(); j++)
		{
			_map[i][j] = 0;
		}
	}

	//fill map with threat values (thread value = urgancy (based on rate of expansion)
	//REMINDER: threat values are not permenant. 0 = no threat, 1-39 means insignificant threat, 40+ means threat worth avoiding (higher number = bigger threat), 100 means critical threat (too late, no possible way to avoid collision)
	for (int i = 0; i < InputData.size(); i++)
	{
		for (int y = InputData[i].GetY(); y <= InputData[i].GetY() + InputData[i].GetLength(); y++)
		{
			for (int x = InputData[i].GetX(); x <= InputData[i].GetX() + InputData[i].GetWidth(); x++)
			{
				if (_map[x][y] < InputData[i].GetUrgency())
					_map[x][y] = InputData[i].GetUrgency();
			}
		}
	}

	//check center of screen for safe passage
	RegionPoints bestSectionRegion;
	bool centreSafePassage = false;

	RegionPoints safeRegion = _centerRegion;

	//iterators used for scanning
	int x = _centerRegion.x0;
	int y = _centerRegion.y0;

	vector<RegionPoints> safeLineSegments;

	//first step, check for start/stop points per row of map
	bool firstMapScanComplete = false;
	bool firstPointFound = false;

	while (!firstMapScanComplete)
	{
		//check map pos for threat, if no worthwhile threat...
		if (_map[x][y] <= 39)
		{
			//if x is at final pixel for the row, or not currently logging a line, start logging a line
			if ((x == _centerRegion.x0 || _map[x - 1][y] > 39) && !firstPointFound)
			{
				firstPointFound = true;
				RegionPoints temp;
				temp.x0 = x;
				temp.y0 = temp.y1 = y;
				safeLineSegments.push_back(temp);
			}
		}
		//otherwise if worthwhile threat, complete the log
		else if (_map[x][y] > 39 && firstPointFound)
		{
			safeLineSegments.back().x1 = x - 1;
			firstPointFound = false;
		}

		// if the current pixel was the last in the row, end any current logs and continue to next row (or end if already on last row)
		if (x == _centerRegion.x1)
		{
			if (firstPointFound)
			{
				safeLineSegments.back().x1 = x;
				firstPointFound = false;
			}

			if (y == _centerRegion.y1)
				firstMapScanComplete = true;
			else
			{
				x = _centerRegion.x0;
				y++;
			}
		}
		//otherwise, increment x
		else
		{
			x++;
		}
	}

	//second step, go through each line and compare with each other for safe zones
	int yCount = 0;
	for (int i = 0; i < safeLineSegments.size(); i++)
	{
		//reset yCount
		yCount = 0;
		bool failedCheck = false;

		//for each safe zone entry, check if it's for the next line. if it isn't, end checking, otherwise check it's validity and increase yCount if acceptable
		for (int j = i + 1; j < safeLineSegments.size() && !failedCheck; j++)
		{

			if (safeLineSegments[j].y0 == (safeLineSegments[i].y0 + 1 + yCount))
			{
				if (safeLineSegments[i].x0 >= safeLineSegments[j].x0 && safeLineSegments[i].x1 <= safeLineSegments[j].x1)
					yCount++;
			}
			else
				failedCheck = true;
		}

		//if no best region set, automatically becomes best region
		if (!RegionPointsSet(bestSectionRegion))
			bestSectionRegion = SetRegionPoints(safeLineSegments[i].x0, safeLineSegments[i].x1, safeLineSegments[i].y0, safeLineSegments[i].y1 + yCount);
		else
		{
			//if region is bigger than currently stored best region (contains more pixels), it becomes new best region
			if (((safeLineSegments[i].x1 - safeLineSegments[i].x0) * (yCount + 1)) > ((bestSectionRegion.x1 - bestSectionRegion.x0) * (bestSectionRegion.y1 - bestSectionRegion.y0)))
				bestSectionRegion = SetRegionPoints(safeLineSegments[i].x0, safeLineSegments[i].x1, safeLineSegments[i].y0, (safeLineSegments[i].y1 + yCount));
		}
	}

	//third step, test best region to see if it's large enough for the fishbot (for now, must be at least 1/3 the size)
	if (bestSectionRegion.x1 - bestSectionRegion.x0 >= ceil((_centerRegion.x1 - _centerRegion.x0) / 2) && bestSectionRegion.y1 - bestSectionRegion.y0 >= ceil((_centerRegion.y1 - _centerRegion.y0) / 2))
	{
		centreSafePassage = true;
		int centrePointX = _centerRegion.x0 + (ceil(_centerRegion.x1 - _centerRegion.x0) / 2);
		int centrePointY = _centerRegion.y0 + (ceil(_centerRegion.y1 - _centerRegion.y0) / 2);

		rectangle(*currentFrame, Point(bestSectionRegion.x0, bestSectionRegion.y0), Point(bestSectionRegion.x1, bestSectionRegion.y1), CV_RGB(50, 255, 20), CV_FILLED);

		//if safe passage is central, move forward, and adjust direction to head for exact centre of safe passage
		if (centrePointX > bestSectionRegion.x0 && centrePointX < bestSectionRegion.x1 && centrePointY > bestSectionRegion.y0 && centrePointY < bestSectionRegion.y1)
		{
			_commandDirections.SetUpDown(0);
			_commandDirections.SetForBack(1);
			_commandDirections.SetLeftRight(0);

			int bestSectionRegionCentreX = (ceil((bestSectionRegion.x1 - bestSectionRegion.x0) / 2) + bestSectionRegion.x0);
			int bestSectionRegionCentreY = (ceil((bestSectionRegion.y1 - bestSectionRegion.y0) / 2) + bestSectionRegion.y0);

			int arrowX = centrePointX;
			int arrowY = centrePointY;

			if (centrePointX > bestSectionRegionCentreX)
			{
				if (centrePointX - bestSectionRegionCentreX > 20)
				{
					_commandDirections.SetLeftRight(-1);
					arrowX -= 20;
				}
			}
			else if (centrePointX < bestSectionRegionCentreX)
			{
				if (bestSectionRegionCentreX - centrePointX > 20)
				{
					_commandDirections.SetLeftRight(1);
					arrowX += 20;
				}
			}

			if (centrePointY > bestSectionRegionCentreY)
			{
				if (centrePointY - bestSectionRegionCentreY > 20)
				{
					_commandDirections.SetUpDown(-1);
					arrowY -= 20;
				}
			}
			else if (centrePointY < bestSectionRegionCentreY)
			{
				if (bestSectionRegionCentreY - centrePointY > 20)
				{
					_commandDirections.SetUpDown(1);
					arrowY += 20;
				}
			}

			if (centrePointX != arrowX || centrePointY != arrowY)
				arrowedLine(*currentFrame, Point(centrePointX, centrePointY), Point(arrowX, arrowY), CV_RGB(200, 200, 0), 6, 8, 0, 0.5);
		}
		//otherwise, adjust so heading towards the found safe passage
		else
		{
			int arrowX = centrePointX;
			int arrowY = centrePointY;

			if (centrePointX > bestSectionRegion.x1)
			{
				_commandDirections.SetLeftRight(-1);
				arrowX -= 40;
			}
			else if (centrePointX < bestSectionRegion.x0)
			{
				_commandDirections.SetLeftRight(1);
				arrowX += 40;
			}
			else
				_commandDirections.SetLeftRight(0);

			if (centrePointY > bestSectionRegion.y1)
			{
				_commandDirections.SetUpDown(-1);
				arrowY -= 40;
			}
			else if (centrePointY < bestSectionRegion.y0)
			{
				_commandDirections.SetUpDown(1);
				arrowY += 40;
			}
			else
				_commandDirections.SetUpDown(0);

			if (centrePointX != arrowX || centrePointY != arrowY)
				arrowedLine(*currentFrame, Point(centrePointX, centrePointY), Point(arrowX, arrowY), CV_RGB(200, 200, 0), 6, 8, 0, 0.5);

			_commandDirections.SetForBack(1);
		}
	}

	//if safe passage center, send commands and return true
	if (centreSafePassage)
	{
		SendDirectionCommands();
		return true;
	}

	//fourth step, if no safe passage center, check whole screen to find safest passage

	//reset/recycle variables where appropriate
	bestSectionRegion = RegionPoints();
	bool safePassage = false;

	safeRegion = RegionPoints();

	RegionPoints fullMap;
	fullMap.x0 = 0;
	fullMap.y0 = 0;
	fullMap.x1 = _screenWidth - 1;
	fullMap.y1 = _screenHeight - 1;

	//iterators used for scanning
	x = fullMap.x0;
	y = fullMap.y0;

	safeLineSegments.clear();

	//fifth step, check for start/stop points per row of full map
	firstMapScanComplete = false;
	firstPointFound = false;

	while (!firstMapScanComplete)
	{
		if (_map[x][y] <= 39)
		{
			if ((x == fullMap.x0 || _map[x - 1][y] > 39) && !firstPointFound)
			{
				firstPointFound = true;
				RegionPoints temp;
				temp.x0 = x;
				temp.y0 = temp.y1 = y;
				safeLineSegments.push_back(temp);
			}
		}
		else if (_map[x][y] > 39 && firstPointFound)
		{
			safeLineSegments.back().x1 = x - 1;
			firstPointFound = false;
		}

		if (x == fullMap.x1)
		{
			if (firstPointFound)
			{
				safeLineSegments.back().x1 = x;
				firstPointFound = false;
			}

			if (y == fullMap.y1)
				firstMapScanComplete = true;
			else
			{
				x = fullMap.x0;
				y++;
			}
		}
		else
		{
			x++;
		}
	}

	//sixth step, go through each line and compare with each other for safe zones
	yCount = 0;
	for (int i = 0; i < safeLineSegments.size(); i++)
	{
		//reset yCount
		yCount = 0;
		bool failedCheck = false;

		//for each safe zone entry, check if it's for the next line. if it isn't, end checking, otherwise check it's validity and increase yCount if acceptable
		for (int j = i + 1; j < safeLineSegments.size() && !failedCheck; j++)
		{

			if (safeLineSegments[j].y0 == (safeLineSegments[i].y0 + 1 + yCount))
			{
				if (safeLineSegments[i].x0 >= safeLineSegments[j].x0 && safeLineSegments[i].x1 <= safeLineSegments[j].x1)
					yCount++;
			}
			else
				failedCheck = true;
		}

		//if no best region set, automatically becomes best region
		if (!RegionPointsSet(bestSectionRegion))
			bestSectionRegion = SetRegionPoints(safeLineSegments[i].x0, safeLineSegments[i].x1, safeLineSegments[i].y0, safeLineSegments[i].y1 + yCount);
		else
		{
			//if region is bigger than currently stored best region (contains more pixels), it becomes new best region
			if (((safeLineSegments[i].x1 - safeLineSegments[i].x0) * (yCount + 1)) > ((bestSectionRegion.x1 - bestSectionRegion.x0) * (bestSectionRegion.y1 - bestSectionRegion.y0)))
				bestSectionRegion = SetRegionPoints(safeLineSegments[i].x0, safeLineSegments[i].x1, safeLineSegments[i].y0, (safeLineSegments[i].y1 + yCount));
		}
	}

	int centrePointX = _centerRegion.x0 + (ceil(_centerRegion.x1 - _centerRegion.x0) / 2);
	int centrePointY = _centerRegion.y0 + (ceil(_centerRegion.y1 - _centerRegion.y0) / 2);

	//seventh step, test best region to see if it's large enough for the fishbot (for now, must be at least 1/3 the size)
	if (bestSectionRegion.x1 - bestSectionRegion.x0 >= ceil((_centerRegion.x1 - _centerRegion.x0) / 2) && bestSectionRegion.y1 - bestSectionRegion.y0 >= ceil((_centerRegion.y1 - _centerRegion.y0) / 2))
	{
		safePassage = true;

		rectangle(*currentFrame, Point(bestSectionRegion.x0, bestSectionRegion.y0), Point(bestSectionRegion.x1, bestSectionRegion.y1), CV_RGB(50, 255, 20), CV_FILLED);
		int arrowX = centrePointX;
		int arrowY = centrePointY;

		_commandDirections.SetUpDown(0);
		_commandDirections.SetForBack(1);
		_commandDirections.SetLeftRight(0);

		if (bestSectionRegion.x1 < centrePointX)
		{
			_commandDirections.SetLeftRight(-1);
			arrowX -= 60;
		}
		else if (bestSectionRegion.x0 > centrePointX)
		{
			_commandDirections.SetLeftRight(1);
			arrowX += 60;
		}
		if (bestSectionRegion.y1 < centrePointY)
		{
			_commandDirections.SetUpDown(-1);
			arrowY -= 60;
		}
		else if (bestSectionRegion.y0 > centrePointY)
		{
			_commandDirections.SetUpDown(1);
			arrowY += 60;
		}

		arrowedLine(*currentFrame, Point(centrePointX, centrePointY), Point(arrowX, arrowY), CV_RGB(200, 200, 0), 6, 8, 0, 0.5);
	}

	//if no safe passage, proceed with emergency actions
	if (!safePassage)
	{
		//if no start time, set start time
		if (_lastTime < 0)
		{
			LARGE_INTEGER li;
			QueryPerformanceCounter(&li);

			_lastTime = (li.QuadPart / 1000.0);
		}
		else
		{
			double lDeltaTime = 0;
			LARGE_INTEGER li;
			QueryPerformanceCounter(&li);

			lDeltaTime = (li.QuadPart / 1000.0) - _lastTime;
			_lastTime = (li.QuadPart / 1000.0);

			_timer += lDeltaTime;
		}

		//if less than 5 seconds, stay still
		if (_timer < _maxIdleTime)
		{
			_commandDirections.SetUpDown(0);
			_commandDirections.SetForBack(0);
			_commandDirections.SetLeftRight(0);
		}
		//otherwise, rotate left
		else
		{
			_commandDirections.SetUpDown(0);
			_commandDirections.SetForBack(0);
			_commandDirections.SetLeftRight(-1);

			arrowedLine(*currentFrame, Point(centrePointX, centrePointY), Point(centrePointX - 20, centrePointY - 20), CV_RGB(200, 200, 0), 6, 8, 0, 0.5);
		}
	}
	else if (_timer > 0)
	{
		_timer = 0;
		_lastTime = -1;
	}

	//send direction commands, then return safePassage value
	SendDirectionCommands();

	//return safePassage;
}*/