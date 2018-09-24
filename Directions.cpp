#include "Directions.h"

using namespace std;

void Directions::SetType(char aType)
{
	_commandType = aType;
};

void Directions::SetDegrees(int aDeg)
{
	_degree = aDeg;
};


void Directions::SetScale(int aScale)
{
	_scale = aScale;
};


void Directions::SetForThrust(int aThrust)
{
	_forwardThrust = aThrust;
};

void Directions::SetRotationDir(int aDir)
{
	_rotationDir = aDir;
};

char Directions::GetType()
{
	return _commandType;
};


int Directions::GetDegrees()
{
	return _degree;
};


int Directions::GetScale()
{
	return _scale;
};


int Directions::GetForThrust()
{
	return _forwardThrust;
};

int Directions::GetRotationDir()
{
	if (_rotationDir > 0)
		return 1;
	else
		return 0;
};

string Directions::GetInstructions()
{
	string temp = "";

	switch (_commandType)
	{
		case 'T':
		{
					temp += _commandType;
					if (_degree < 10)
					{
						temp += "00";
						temp += _degree;
					}
					else if (_degree < 100)
					{
						temp += "0";
						temp += _degree;
					}
					else
						temp += _degree;

					if (_scale < 10)
					{
						temp += "0";
						temp += _scale;
					}
					else
						temp += _scale;

					if (_forwardThrust < 0)
						temp += "0";
					else
						temp += "1";

					if (_forwardThrust < 10 && _forwardThrust > -10)
					{
						temp += "0";
						temp += _forwardThrust;
					}
					else
						temp += _forwardThrust;

					break;
		}
		case 'R':
		{
					temp += _commandType;
					if (_degree < 10)
					{
						temp += "00";
						temp += _degree;
					}
					else if (_degree < 100)
					{
						temp += "0";
						temp += _degree;
					}
					else
						temp += _degree;

					if (_scale < 10)
					{
						temp += "0";
						temp += _scale;
					}
					else
						temp += _scale;

					if (_rotationDir > 0)
						temp += "1";
					else
						temp += "0";

					break;
		}
		case 'S':
		{
					temp = "S";
					break;
		}
		default: { }
	}

	return temp;
}

void Directions::ClearDirections()
{
	_commandType = 'T';
	_degree = 0;
	_scale = 0;
	_forwardThrust = 0;
	_rotationDir = 0;
}