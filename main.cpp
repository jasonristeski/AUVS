//#pragma once
#include "AttractRepulseNav.h"
#include "Tracker.h"
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
bool navDebug = true;
bool navigating = true;

//**********************USER CONTROLS************************************
//PRESS 'p' or 'f' TO PAUSE or 'q' to quit
//ONCE PAUSED:
//'p': unpause
//'f': advance to next frame
//'d': toggle detection overlay (Records detected boxes and overlays them onscreen)
//'t': toggle obstacle Tracing (Draws a line following an obstacles centrepoint)
//'c': clear centrepoint trace and detection history
//'w': toggle tracker debug output

ofstream writer2;
clock_t t2;
int main()
{
	writer2.open("perf.csv");
	writer2 << "Detection before, detection after,tracking after,NAV after" << std::endl;
	Detector* objectDetector = new Detector("botTest3.mp4"); // create detector 
	Tracker myTracker(objectDetector); //Create Tracker
	AttractRepulseNav myNavigator = AttractRepulseNav(320, 240, 10, 10, 30, 10, 10, 30); //Create Navigator
	//constructor: (width, height, front width, front height, side width, side height, top width, top height)

	myNavigator.InitialSetup();//set up nav (build map space, calculate central region, flag as initialized)

	//variables used for storing data that is used by navigator
	vector<NavObject> beaconObjectData = vector<NavObject>();
	vector<NavObject> objectData = vector<NavObject>();
	Mat currentFrame = Mat(240, 320, CV_8UC3, CV_RGB(0, 0, 0));

	//****************TOGGLE DEBUG OUTPUT SETTINGS**********************
	//myTracker.ToggleDebug(); //write debug output to console
	myTracker.SetShowDisplay(); //turn on to display tracking output
	//myTracker.ToggleTracing(); //turn on to trace the centrepoint of tracked objects
	//myTracker.ToggleDetectionOverlay(); //turn on to overlay the tracking output with detection history
	myTracker.ToggleDetectionWindows(); //turn on detection windows
										//myTracker.setShowDisplay(); //turn on to show detection display windows

										//*********************RUN MAIN LOOP*************************************************
	double time = 0;
	while (!paused)
	{

		beaconObjectData.clear();
		objectData.clear();
		currentFrame = CV_RGB(255, 0, 0);
		

		//time = clock();
		writer2 << 1000 * clock() / CLOCKS_PER_SEC << ",";
		objectDetector->Detect();
		writer2 << 1000 * clock()  / CLOCKS_PER_SEC << ",";
  		myTracker.RefreshAll(); //Refresh tracker (update obstacles and display if appropriate flags are set)
		writer2 << 1000 * clock()  / CLOCKS_PER_SEC << ",";
		for (int i = 0; i < myTracker.myNavObjects.size(); i++)
		{
			objectData.push_back(myTracker.myNavObjects[i]);
		}

		objectData = myTracker.myNavObjects;
		//objectData = myTracker.myNavObjects;
		//Fake/dummy data
		//fake beacon
		//beaconObjectData.push_back(NavObject(0, 0, 10, 10, -1));
		//fake objects
		//objectData.push_back(NavObject(5, 5, 20, 40, 70));
		//objectData.push_back(NavObject(0,0,310,230,1));
		//objectData.push_back(NavObject(106, 78, 160, 3, 50));
		//objectData.push_back(NavObject(135, 78, 160, 28, 51));
		//objectData.push_back(NavObject(70, 0, 160, 50, 44));
		//objectData.push_back(NavObject(70, 180, 220, 45, 44));

		if (navigating)
		{
			//if debug mode, run debug function, otherwise read non-debug function
			if (navDebug) //if tracker has a "IsDebug()" function, could be tied to te tracker debug toggle
			{
				myNavigator.DebugDecideDirections(&currentFrame, beaconObjectData, objectData);
				namedWindow("Navigator", WINDOW_NORMAL);
				imshow("Navigator", currentFrame);
			}
			else
				myNavigator.DecideDirections(beaconObjectData, objectData);
			writer2 << 1000 * clock() / CLOCKS_PER_SEC << ",";
		}
                
                writer2 << std::endl;
		

		userInput = cvWaitKey(1); //get user input

		if (userInput == 'f' || userInput == 'p') //pause video if user enters 'f' or 'p'
			paused = true;
		else if (userInput == 'q') //exit video if user press 'q'
			return 0;


		while (paused) //loop while paused
		{
			userInput = cvWaitKey(0); //get user input

									  //run function redraw output if user changes parameter
			if (userInput == 'c') //if user presses 'c', clear detection and tracking traces
				myTracker.ClearTrace();
			if (userInput == 'd') //if user presses 'd', toggle detextion overlay
				myTracker.ToggleDetectionOverlay();
			if (userInput == 't') //if user presses 't', toggle tracking tracing
				myTracker.ToggleTracing();
			if (userInput == 'w') //if user presses 'w', toggle debug output
				myTracker.ToggleDebug();
			if (userInput == 'c' || userInput == 'd' || userInput == 't')
				myTracker.PrintResults();

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
