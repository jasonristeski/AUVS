/*	------------------------------------
DIRECTIONS
------------------------------------
Description:
Stores flags indicating directions calculated by Nav. Modified using functions in order to ensure no contradictory flag sets occur.
------------------------------------
Fields:
_up (double)
_down (double)
_left (double)
_right (double)
_forward (double)
_backward (double)
Functions:
public void SetUpDown(int DirValue)
public void SetLeftRight(int DirValue)
public void SetForBack(int DirValue)
public void ClearDirections()
public vector<bool> GetDirections()
------------------------------------*/

#pragma once
#include <vector>

class AdvancedDirections
{
private:
	//Direction flags
	double _up;
	double _down;
	double _left;
	double _right;
	double _forward;
	double _backward;
public:
	AdvancedDirections()
	{
		ClearDirections();
	}
	//DirValue indicates what direction should be set; less than 0 means first direction, more than 0 means second direction, 0 means neither.
	//e.g. For SetUpDown, <0 = up true down false, >0 = down true up false, 0 = both false

	void SetUpDown(int DirValue);
	void SetLeftRight(int DirValue);
	void SetForBack(int DirValue);

	//Sets all fields to false
	void ClearDirections();

	//Returns bool vector containing all flag values (in order of up, down, left, right, forward, backward)
	std::vector<double> GetDirections();
};