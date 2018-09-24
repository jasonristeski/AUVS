#include "AdvancedDirections.h"

using namespace std;

void AdvancedDirections::SetUpDown(int DirValue)
{
	_up = 0.0;
	_down = 0.0;

	if (DirValue > 0)
		_down = static_cast<double>(DirValue);
	else if (DirValue < 0)
		_up = static_cast<double>(DirValue * -1);
}

void AdvancedDirections::SetLeftRight(int DirValue)
{
	_left = 0.0;
	_right = 0.0;

	if (DirValue > 0)
		_right = static_cast<double>(DirValue);
	else if (DirValue < 0)
		_left = static_cast<double>(DirValue * -1);
}

void AdvancedDirections::SetForBack(int DirValue)
{
	_forward = 0.0;
	_backward = 0.0;

	if (DirValue > 0)
		_forward = static_cast<double>(DirValue);
	else if (DirValue < 0)
		_backward = static_cast<double>(DirValue * -1);
}

vector<double> AdvancedDirections::GetDirections()
{
	vector<double> temp{ _up,
		_down,
		_left,
		_right,
		_forward,
		_backward
	};

	return temp;
}

void AdvancedDirections::ClearDirections()
{
	_up = 0.0;
	_down = 0.0;
	_left = 0.0;
	_right = 0.0;
	_forward = 0.0;
	_backward = 0.0;
}