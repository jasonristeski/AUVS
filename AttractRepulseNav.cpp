//#pragma once
#include "AttractRepulseNav.h"
#include <iostream>
#include<sstream>
#include <math.h>
#include "PhysVector2D.h"
#include <time.h>
#include "Comms.h"

#define PI 3.14159265358979323846

using namespace std;
using namespace cv;


clock_t t;
float currentTime = 0, updatePeriod = 1000, lastUpdate = 0;
bool updateSerial = true;
bool commsEnabled = false;

//function sourced from http://flassari.is/2008/11/line-line-intersection-in-cplusplus/
Point* Intersection(Point p1, Point p2, Point p3, Point p4) {
	// Store the values for fast access and easy
	// equations-to-code conversion
	float x1 = p1.x, x2 = p2.x, x3 = p3.x, x4 = p4.x;
	float y1 = p1.y, y2 = p2.y, y3 = p3.y, y4 = p4.y;

	float d = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
	// If d is zero, there is no intersection
	if (d == 0) return NULL;

	// Get the x and y
	float pre = (x1*y2 - y1*x2), post = (x3*y4 - y3*x4);
	float x = (pre * (x3 - x4) - (x1 - x2) * post) / d;
	float y = (pre * (y3 - y4) - (y1 - y2) * post) / d;

	// Check if the x and y coordinates are within both lines
	if (x < min(x1, x2) || x > max(x1, x2) ||
		x < min(x3, x4) || x > max(x3, x4)) return NULL;
	if (y < min(y1, y2) || y > max(y1, y2) ||
		y < min(y3, y4) || y > max(y3, y4)) return NULL;

	// Return the point of intersection
	Point* ret = new Point();
	ret->x = x;
	ret->y = y;
	return ret;
}

void AttractRepulseNav::InitialSetup()
{
	if (commsEnabled)
	{
		Comms communicator;
		//communicator.ReadyComm();
	}
		
        
	map = Mat(screenWidth, screenHeight, CV_8U);
	map = Scalar(100);
	
	halfHeight = ceil(screenHeight / 2);
	halfWidth = ceil(screenWidth / 2);
	
	//calculate max distance possible on screen
	PhysVector2D temp = PhysVector2D(0, 0);
	PhysVector2D temp2 = PhysVector2D(0 - halfWidth, 0 - halfHeight);

	maxDistance = temp.Distance(temp2);

	cout << "map width is " << screenWidth << "\n";
	cout << "map height is " << screenHeight << "\n\n";

	_initialSetupComplete = true;
}

float AttractRepulseNav::GetPollValue()
{
	//NOT IMPLEMENTED - returns 0.2

	//call comms, get speed
	//using speed, calculate next polling value and return it (min. x, max. y)
	float result = (float)0.2;
	return result;
}

/*ORIGINAL CENTER-ONLY NAV!~~~~!

bool AttractRepulseNav::DecideDirections(vector<NavObject> BeaconData, vector<NavObject> InputData)
{
//check initialized (critical error avoidance)
if (!_initialSetupComplete)
{
cout << "NAV: Not initialized\n";
return false;
}

PhysVector2D result = PhysVector2D();
PhysVector2D centerPoint = PhysVector2D(0, 0);

int totalThreat = 0;
int objCount = 0;

for (int i = 0; i < InputData.size(); i++)
{
PhysVector2D obj = PhysVector2D(0 - (halfWidth - ceil(InputData[i].GetX() + (InputData[i].GetWidth() / 2))), 0 + (halfHeight - ceil(InputData[i].GetY() + (InputData[i].GetWidth() / 2))));
PhysVector2D objInvert = PhysVector2D(0 + (halfWidth - ceil(InputData[i].GetX() + (InputData[i].GetWidth() / 2))), 0 - (halfHeight - ceil(InputData[i].GetY() + (InputData[i].GetWidth() / 2))));
PhysVector2D objDir = PhysVector2D((objInvert).GetNormalised());
float dist = centerPoint.Distance(objInvert);

objDir = objDir *  InputData[i].GetUrgency() * (((maxDistance - dist) / maxDistance));

result = result + objDir;

totalThreat += InputData[i].GetUrgency();
objCount++;
}

//PRODUCE COMMS DATA AND PASS TO COMMS
float resultRadian = atan2(result.GetY(), result.GetX());
float resultDegree = floor(resultRadian * 180 / PI);

//_maxdistance/15 means distance is capped at 15 (15 being literally to the edge of the screen), floor rounding means if the distance change doesn't at least cover a miniscule range of the screen, it won't bother moving at all
int scale = floor((centerPoint.Distance(result) / (maxDistance / 15)));

//z thrust % is calculated based on 99 - average treat
int forwardThrustPerc = 99;

if (objCount > 0 && totalThreat > 0)
forwardThrustPerc -= ceil(totalThreat / objCount);

return true;
}

*/

bool AttractRepulseNav::DecideDirections(vector<NavObject> BeaconData, vector<NavObject> InputData)
{
	//check initialized (critical error avoidance)
	if (!_initialSetupComplete)
	{
		cout << "NAV: Not initialized\n";
		return false;
	}

	if (turnActive)
	{
		double lDeltaTime = clock() - lastTurnTime;
		lastTurnTime = clock();

		turnTimer += lDeltaTime;

		if (turnTimer >= turnMaxIdleTime)
		{
			turnTimer = 0;
			lastTime = -1;
			turnActive = false;
		}
	}
	else
	{
		PhysVector2D result = PhysVector2D();
		PhysVector2D centerPoint = PhysVector2D(0, 0);
		bool NotSafeToMove = false;
		bool TurnTriggered = false;

		int totalThreat = 0;
		int objCount = 0;

		for (int i = 0; i < InputData.size(); i++)
		{
			if (InputData[i].GetWidth() >= floor(screenWidth * 0.95) && InputData[i].GetLength() >= floor(screenHeight * 0.95)) //!~~1
				NotSafeToMove = true;

			PhysVector2D objInvert;

			if (InputData[i].GetAttract())
				objInvert = PhysVector2D(0 - (halfWidth - ceil(InputData[i].GetX() + (InputData[i].GetWidth() / 2))), 0 + (halfHeight - ceil(InputData[i].GetY() + (InputData[i].GetLength() / 2))));
			else
				objInvert = PhysVector2D(0 + (halfWidth - ceil(InputData[i].GetX() + (InputData[i].GetWidth() / 2))), 0 - (halfHeight - ceil(InputData[i].GetY() + (InputData[i].GetLength() / 2))));

			PhysVector2D objDir = PhysVector2D((objInvert).GetNormalised());

			float dist = 0;

			//if center is within boundaries of object, measure distance from object center to center
			if (halfWidth >= InputData[i].GetX() && halfWidth <= InputData[i].GetX() + InputData[i].GetWidth() &&
				halfHeight >= InputData[i].GetY() && halfHeight <= InputData[i].GetY() + InputData[i].GetLength())
				//if object center IS at the center, set distance to 19 (as long as the object threat is high enough, it WILL cause the bot to move slightly)
			if (objInvert.GetX() == objInvert.GetY() == 0.0)
				dist = centerPoint.Distance(objInvert);
			else
				dist = 19;
			//otherwise, use the CLOSEST EDGE POINT of the object when measuring distance from the center
			else
			{
				//create points for each corner of object, center of screen and center of object
				Point center = Point(0, 0);
				Point objPt = Point(objInvert.GetX(), objInvert.GetY());

				Point a = Point(objInvert.GetX() - ceil(InputData[i].GetWidth() / 2), objInvert.GetY() + ceil(InputData[i].GetLength() / 2));
				Point b = Point(objInvert.GetX() + ceil(InputData[i].GetWidth() / 2), objInvert.GetY() - ceil(InputData[i].GetLength() / 2));
				Point c = Point(objInvert.GetX() + ceil(InputData[i].GetWidth() / 2), objInvert.GetY() - ceil(InputData[i].GetLength() / 2));
				Point d = Point(objInvert.GetX() - ceil(InputData[i].GetWidth() / 2), objInvert.GetY() + ceil(InputData[i].GetLength() / 2));

				bool intersectFound = false;
				Point* intersect = NULL;
				int count = 0;

				//cycle through object edge lines to find an intersection (already confirmed there will be one!), once found, exit loop
				while (!intersectFound)
				{
					switch (count)
					{
					case 0:
					{
							  intersect = Intersection(center, objPt, a, b);
							  break;
					}
					case 1:
					{
							  intersect = Intersection(center, objPt, b, c);
							  break;
					}
					case 2:
					{
							  intersect = Intersection(center, objPt, c, d);
							  break;
					}
					case 3:
					{
							  intersect = Intersection(center, objPt, d, a);
							  break;
					}
					default:{}
					}

					if (intersect != NULL)
						intersectFound = true;

					count++;
					if (count >= 4)
						intersectFound = true;
				}

				//if no intersect point found, revert to using center, otherwise...
				if (intersect == NULL)
					dist = centerPoint.Distance(objInvert);
				else
				{
					//create a PhysVector using point results, then measure and store distance
					PhysVector2D intPoint = PhysVector2D(intersect->x, intersect->y);
					dist = centerPoint.Distance(intPoint);
				}
			}

			objDir = objDir * InputData[i].GetUrgency() * (((maxDistance - dist) / maxDistance));

			if (InputData[i].GetDynamic())
			{
				objDir * 2;

				if (InputData[i].GetXVel() > InputData[i].GetYVel())
				{
					objDir = objDir * PhysVector2D(1,-1);
				}
				else if (InputData[i].GetXVel() < InputData[i].GetYVel())
				{
					objDir = objDir * PhysVector2D(-1, 1);
				}
				else
				{
					objDir = objDir * -1;
				}
			}

			result = result + objDir;

			totalThreat += InputData[i].GetUrgency();
			objCount++;
		}

		for (int i = 0; i < BeaconData.size(); i++)
		{
			if (BeaconData[i].GetWidth() >= floor(screenWidth * 0.75) && BeaconData[i].GetLength() >= floor(screenHeight * 0.75)) //!~~2
				TurnTriggered = true;

			PhysVector2D objInvert;

			objInvert = PhysVector2D(0 - (halfWidth - ceil(BeaconData[i].GetX() + (BeaconData[i].GetWidth() / 2))), 0 + (halfHeight - ceil(BeaconData[i].GetY() + (BeaconData[i].GetLength() / 2))));

                        
			PhysVector2D objDir = PhysVector2D((objInvert).GetNormalised());

			float dist = 0;

			//if center is within boundaries of object, measure distance from object center to center
			if (halfWidth >= InputData[i].GetX() && halfWidth <= InputData[i].GetX() + InputData[i].GetWidth() &&
				halfHeight >= InputData[i].GetY() && halfHeight <= InputData[i].GetY() + InputData[i].GetLength())
				//if object center IS at the center, set distance to 19 (as long as the object threat is high enough, it WILL cause the bot to move slightly)
			if (objInvert.GetX() == objInvert.GetY() == 0.0)
				dist = centerPoint.Distance(objInvert);
			else
				dist = 19;

			objDir = objDir *  InputData[i].GetUrgency() * (((maxDistance - dist) / maxDistance));

			result = result + objDir;
		}

		//if distance goes past screen, this should bring it closer to scale again
		if (centerPoint.Distance(result) > maxDistance)
		{
			float usedPerc = maxDistance / centerPoint.Distance(result);
			result = PhysVector2D(result.GetX() * usedPerc, result.GetY() * usedPerc);
		}

		if (!TurnTriggered)
		{
			//PRODUCE COMMS DATA AND PASS TO COMMS
			float resultRadian = atan2(result.GetY(), result.GetX());
			float resultDegree = floor(resultRadian * 180 / PI);

			int scale = 0;

			if (!NotSafeToMove)
				//_maxdistance/15 means distance is capped at 15 (15 being literally to the edge of the screen), floor rounding means if the distance change doesn't at least cover a miniscule range of the screen, it won't bother moving at all
				scale = floor((centerPoint.Distance(result) / (maxDistance / 15)));

			//z thrust % is calculated based on 99 - average treat
			int forwardThrustPerc = 99;

			if (NotSafeToMove)
				forwardThrustPerc = 0;
			else
			if (objCount > 0 && totalThreat > 0)
				forwardThrustPerc -= ceil(totalThreat / objCount);

			directions.SetType('T');
			directions.SetDegrees(resultDegree);
			directions.SetScale(scale);
			directions.SetForThrust(forwardThrustPerc);
                        //this is where you could send directions Andrew
		}
		else
		{
			turnActive = true;

			lastTurnTime = clock();

			directions.SetType('R');
			directions.SetRotationDir(0);
			directions.SetScale(5);
			directions.SetDegrees(180);
                        //this is where you could send directions Andrew
		}

		return true;
	}
}

//UNCOMMENT COMMENTS TO INCLUDE TEXT-FEEDBACK PER OBJECT HANDLED
bool AttractRepulseNav::DebugDecideDirections(Mat* currentFrame, vector<NavObject> BeaconData, vector<NavObject> InputData)
{
	//check initialized (critical error avoidance)
	if (!_initialSetupComplete)
	{
		cout << "NAV: Not initialized\n";
		return false;
	}
        
        //update current time
        currentTime = 1000 * clock()  / CLOCKS_PER_SEC;
        

	if (turnActive)
	{
		double lDeltaTime = clock() - lastTime;
		lastTurnTime = clock();

		turnTimer += lDeltaTime;

		if (turnTimer >= turnMaxIdleTime)
		{
			turnTimer = 0;
			lastTurnTime = -1;
			turnActive = false;
		}
	}
	else
	{
		PhysVector2D result = PhysVector2D();
		PhysVector2D centerPoint = PhysVector2D(0, 0);

		int totalThreat = 0;
		int objCount = 0;
		bool NotSafeToMove = false;
		bool TurnTriggered = false;

		/*
		TO DO
		-At the moment distance is center of object to center of screen - instead, it should be closest edge of object to center
		-This will be added soon as it drastically improves the accuracy of the force weighting - the weighting will be of acceptable standard in the meantime
		-Add attraction (essentially invert the values applied to result
		-Add beacons (beacons attract with a higher value than normal, help to center bot's x/y position in environment after it comes into view) and turn state (once beacon has x threat/expansion, trigger turning and ensure fish has turned fully before calculating new navigational directions)
		*/
		for (int i = 0; i < InputData.size(); i++)
		{
			if (InputData[i].GetWidth() >= floor(screenWidth * 0.95) && InputData[i].GetLength() >= floor(screenHeight * 0.95))
				NotSafeToMove = true;

			if (commsEnabled)
			{
				Rect obstacleBox = Rect(Point(InputData[i].GetX(), InputData[i].GetY()), Point(InputData[i].GetX() + InputData[i].GetWidth(), InputData[i].GetY() + InputData[i].GetLength()));
				//communicator.SendRect(obstacleBox);
			}
                        
			rectangle(*currentFrame, Point(InputData[i].GetX(), InputData[i].GetY()), Point(InputData[i].GetX() + InputData[i].GetWidth(), InputData[i].GetY() + InputData[i].GetLength()), CV_RGB(255, 255, 0), CV_FILLED);
			rectangle(*currentFrame, Point(InputData[i].GetX(), InputData[i].GetY()), Point(InputData[i].GetX() + InputData[i].GetWidth(), InputData[i].GetY() + InputData[i].GetLength()), CV_RGB(20, 20, 20));
			//PhysVector2D obj = PhysVector2D(0 - (halfWidth - ceil(InputData[i].GetX() + (InputData[i].GetWidth() / 2))), 0 + (halfHeight - ceil(InputData[i].GetY() + (InputData[i].GetLength() / 2))));
			PhysVector2D objInvert = PhysVector2D(0 + (halfWidth - ceil(InputData[i].GetX() + (InputData[i].GetWidth() / 2))), 0 - (halfHeight - ceil(InputData[i].GetY() + (InputData[i].GetLength() / 2))));
			PhysVector2D objDir = PhysVector2D((objInvert).GetNormalised());


			float dist = 0;

			//if center is within boundaries of object, measure distance from object center to center
			if (halfWidth >= InputData[i].GetX() && halfWidth <= InputData[i].GetX() + InputData[i].GetWidth() &&
				halfHeight >= InputData[i].GetY() && halfHeight <= InputData[i].GetY() + InputData[i].GetLength())
				//if object center IS at the center, set distance to 19 (as long as the object threat is high enough, it WILL cause the bot to move slightly)
			if (objInvert.GetX() == objInvert.GetY() == 0.0)
				dist = centerPoint.Distance(objInvert);
			else
				dist = 19;
			//otherwise, use the CLOSEST EDGE POINT of the object when measuring distance from the center
			else
			{
				//create points for each corner of object, center of screen and center of object
				Point center = Point(0, 0);
				Point objPt = Point(objInvert.GetX(), objInvert.GetY());

				Point a = Point(objInvert.GetX() - ceil(InputData[i].GetWidth() / 2), objInvert.GetY() + ceil(InputData[i].GetLength() / 2));
				Point b = Point(objInvert.GetX() + ceil(InputData[i].GetWidth() / 2), objInvert.GetY() - ceil(InputData[i].GetLength() / 2));
				Point c = Point(objInvert.GetX() + ceil(InputData[i].GetWidth() / 2), objInvert.GetY() - ceil(InputData[i].GetLength() / 2));
				Point d = Point(objInvert.GetX() - ceil(InputData[i].GetWidth() / 2), objInvert.GetY() + ceil(InputData[i].GetLength() / 2));

				bool intersectFound = false;
				Point* intersect = NULL;
				int count = 0;

				//cycle through object edge lines to find an intersection (already confirmed there will be one!), once found, exit loop
				while (!intersectFound)
				{
					switch (count)
					{
					case 0:
					{
							  intersect = Intersection(center, objPt, a, b);
							  break;
					}
					case 1:
					{
							  intersect = Intersection(center, objPt, b, c);
							  break;
					}
					case 2:
					{
							  intersect = Intersection(center, objPt, c, d);
							  break;
					}
					case 3:
					{
							  intersect = Intersection(center, objPt, d, a);
							  break;
					}
					default:{}
					}

					if (intersect != NULL)
						intersectFound = true;

					count++;
					if (count >= 4)
						intersectFound = true;
				}

				//if no intersect point found, revert to using center, otherwise...
				if (intersect == NULL)
					dist = centerPoint.Distance(objInvert);
				else
				{
					//create a PhysVector using point results, then measure and store distance
					PhysVector2D intPoint = PhysVector2D(intersect->x, intersect->y);
					dist = centerPoint.Distance(intPoint);
				}
			}

			//cout << dist << " is distance between object and center.\n";

			//cout << "the applied object weighting is " << (InputData[i].GetUrgency() * (((maxDistance - dist) / maxDistance))) << "\n";

			//float radian = atan2(objDir.GetY(), objDir.GetX());
			//float degree = radian * 180 / PI;

			//cout << "the angle the object's repel force applies to the center is (radians) " << radian << " or (degrees)" << degree << "\n";

			objDir = objDir *  InputData[i].GetUrgency() * (((maxDistance - dist) / maxDistance));

			//if object is dynamic, double the bearing it generates (so it has higher priority over other objects), and adjust bearing to compensate
			if (InputData[i].GetDynamic())
			{
				objDir * 2;

				if (InputData[i].GetXVel() > InputData[i].GetYVel())
				{
					objDir = objDir * PhysVector2D(1, -1);
				}
				else if (InputData[i].GetXVel() < InputData[i].GetYVel())
				{
					objDir = objDir * PhysVector2D(-1, 1);
				}
				else
				{
					objDir = objDir * -1;
				}
			}

			//radian = atan2(objDir.GetY(), objDir.GetX());
			//degree = radian * 180 / PI;

			//cout << "after applying repel force, the angle the object's repel force applies to the center is (radians) " << radian << " or (degrees)" << degree << "\n";

			arrowedLine(*currentFrame,
				Point(ceil(InputData[i].GetX() + (InputData[i].GetWidth() / 2)), ceil(InputData[i].GetY() + (InputData[i].GetLength() / 2))),
				Point(ceil(InputData[i].GetX() + (InputData[i].GetWidth() / 2)) + objDir.GetX(), ceil(InputData[i].GetY() + (InputData[i].GetLength() / 2)) - objDir.GetY()),
				CV_RGB(255, 0, 0), 2, 1, 0, 0.15);

			result = result + objDir;
			//cout << "------------------------------\n\n";
                        
                    
			totalThreat += InputData[i].GetUrgency();
			objCount++;
		}

		for (int i = 0; i < BeaconData.size(); i++)
		{
			if (BeaconData[i].GetWidth() >= floor(screenWidth * 0.75) && BeaconData[i].GetLength() >= floor(screenHeight * 0.75))
				TurnTriggered = true;

			PhysVector2D objInvert;

			objInvert = PhysVector2D(0 - (halfWidth - ceil(BeaconData[i].GetX() + (BeaconData[i].GetWidth() / 2))), 0 + (halfHeight - ceil(BeaconData[i].GetY() + (BeaconData[i].GetLength() / 2))));

			PhysVector2D objDir = PhysVector2D((objInvert).GetNormalised());

			float dist = 0;

			//if center is within boundaries of object, measure distance from object center to center
			if (halfWidth >= BeaconData[i].GetX() && halfWidth <= BeaconData[i].GetX() + BeaconData[i].GetWidth() &&
				halfHeight >= BeaconData[i].GetY() && halfHeight <= BeaconData[i].GetY() + BeaconData[i].GetLength())
				//if object center IS at the center, set distance to 19 (as long as the object threat is high enough, it WILL cause the bot to move slightly)
			if (objInvert.GetX() == objInvert.GetY() == 0.0)
				dist = centerPoint.Distance(objInvert);
			else
				dist = 19;

			objDir = objDir *  BeaconData[i].GetUrgency() * (((maxDistance - dist) / maxDistance));

			result = result + objDir;
		}

		//if distance goes past screen, this should bring it closer to scale again
		if (centerPoint.Distance(result) > maxDistance)
		{
			float usedPerc = maxDistance / centerPoint.Distance(result);
			result = PhysVector2D(result.GetX() * usedPerc, result.GetY() * usedPerc);
		}

		if (!NotSafeToMove)
                {
                    Point a =   Point(halfWidth,halfHeight);
                    Point b = Point(halfWidth + result.GetX(),halfHeight - result.GetY());
                        
                    arrowedLine(*currentFrame,a,b,CV_RGB(100, 100, 255), 2, 1, 0, 0.15);
                    
					if (commsEnabled)
					{
						//communicator.SendLine(a.x, a.y, b.x, b.y, 1);
					}
                }
			
		else if (TurnTriggered)
			arrowedLine(*currentFrame,
			Point(halfWidth - 10, halfHeight),
			Point(halfWidth + 20, halfHeight),
			CV_RGB(200, 100, 100), 2, 1, 0, 0.15);
		else
			rectangle(*currentFrame,
			Point(centerPoint.GetX() - 5, centerPoint.GetY() - 5),
			Point(10, 10),
			CV_RGB(100, 100, 255), CV_FILLED);

		if (!TurnTriggered)
		{
			//PRODUCE COMMS DATA AND PASS TO COMMS
			float resultRadian = atan2(result.GetY(), result.GetX());
			float resultDegree = floor(resultRadian * 180 / PI);
			//cout << "Result direction's angles are (radians) " << resultRadian << " or (degrees)" << resultDegree << "\n";

			int scale = 0;

			if (!NotSafeToMove)
				//_maxdistance/15 means distance is capped at 15 (15 being literally to the edge of the screen), floor rounding means if the distance change doesn't at least cover a miniscule range of the screen, it won't bother moving at all
				scale = floor((centerPoint.Distance(result) / (maxDistance / 15)));

			//cout << "the scale is " << scale << "\n";

			int forwardThrustPerc = 99;

			if (NotSafeToMove)
				forwardThrustPerc = 0;
			else
			if (objCount > 0 && totalThreat > 0)
				forwardThrustPerc -= ceil(totalThreat / objCount);
			//cout << "forward thrust power is calculated as " << forwardThrustPerc << "%\n";

			directions.SetType('T');
			directions.SetDegrees(resultDegree);
			directions.SetScale(scale);
			directions.SetForThrust(forwardThrustPerc);
		}
		else
		{
			turnActive = true;

			lastTurnTime = clock();

			directions.SetType('R');
			directions.SetRotationDir(0);
			directions.SetScale(5);
			directions.SetDegrees(180);
		}
	}

	return true;
}