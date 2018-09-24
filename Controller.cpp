//#pragma once
#include "AttractRepulseNav.h"
#include "Tracker.h"
#include "BeaconTracker.h"
#include "Detector.h"
#include "NavObject.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include <iostream>
#include <math.h>
#include <time.h>
#include <fstream>


char userInput;
bool paused = false;
bool showNav = false; //toggles navigation window

//**********************USER CONTROLS************************************
//PRESS 'p' or 'f' TO PAUSE or 'q' to quit
//ONCE PAUSED:
//'p': unpause
//'f': advance to next frame
//'d': toggle detection overlay (Records detected boxes and overlays them onscreen)
//'t': toggle obstacle Tracing (Draws a line following an obstacles centrepoint)
//'c': clear centrepoint trace and detection history
//'w': toggle tracker debug output

clock_t t2;
int main()
{
	Detector* objectDetector = new Detector("Footage/2.mp4"); //Create detector
	Tracker staticTracker(objectDetector, "static"); //Create Static Tracker
	Tracker dynamicTracker(objectDetector, "dynamic"); //Create Dynamic Tracker
	AttractRepulseNav myNavigator = AttractRepulseNav(320, 240, 10, 10, 30, 10, 10, 30); //Create Navigator

	myNavigator.InitialSetup();//set up nav (build map space, calculate central region, flag as initialized)

	//variables used for storing data that is used by navigator
	vector<NavObject> beaconObjectData = vector<NavObject>();
	vector<NavObject> objectData = vector<NavObject>();
	Mat currentFrame = Mat(240, 320, CV_8UC3, CV_RGB(0, 0, 0));

	//****************TOGGLE DEBUG OUTPUT SETTINGS**********************
	//turn on to display tracking output
	staticTracker.SetShowDisplay(); 
	dynamicTracker.SetShowDisplay(); 
	objectDetector->toggleDisplay();
	showNav = true;
	//objectDetector->ToggleDebug();

	//*********************RUN MAIN LOOP*************************************************
	double time = 0;
	while (1)
	{
		beaconObjectData.clear();
		objectData.clear();
		currentFrame = CV_RGB(255, 0, 0);

		objectDetector->Detect();

		//Refresh trackers (update obstacles and display if appropriate flags are set)
		dynamicTracker.RefreshAll();
		staticTracker.RefreshAll(); 

		//get object data to send to navigator
		objectData = staticTracker.myNavObjects;
		objectData.insert( objectData.end(), dynamicTracker.myNavObjects.begin(), dynamicTracker.myNavObjects.end());

		//call navigator to update navigation bearing
		myNavigator.DebugDecideDirections(&currentFrame, beaconObjectData, objectData);
		
		//draw navigator output if applicable
		if (showNav)
		{
			namedWindow("Navigator", WINDOW_NORMAL);
			imshow("Navigator", currentFrame);
		}


		userInput = cvWaitKey(1); //get user input

		if (userInput == 'f' || userInput == 'p') //pause video if user enters 'f' or 'p'
			paused = true;
		else if (userInput == 'q') //exit video if user press 'q'
			return 0;


		while (paused) //loop while paused
		{
			userInput = cvWaitKey(0); //get user input

			//resume, advance, or quit depending on user input
			if (userInput == 'p') //if user presses p, resume video
				paused = false;
			if (userInput == 'f') //if user presses 'f' advance one frame
				break;
			if (userInput == 'q') //if user presses 'q', quit
				return 0;
		}
	}
}
