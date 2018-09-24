/*	------------------------------------
	DIRECTIONS
	------------------------------------
	Description:
	Stores navigational data generated via Navigator modules/classes. Can be used to generate and return a readable/usable command string
	------------------------------------
	Fields:
		commandType (char)
		degree (int)
		scale (int)
		forwardThrust (int)
		rotationDir (int)
	Functions:
	void SetType(char aType);
	void SetDegrees(int aDeg);
	void SetScale(int aScale);
	void SetForThrust(int aThrust);
	void SetRotationDir(int _aDir);
	char GetType();
	int GetDegrees();
	int GetScale();
	int GetForThrust();
	int GetRotationDir();


	//Sets all fields to 0
	void ClearDirections();

	//Returns string of instructions
	std::string GetInstructions();
------------------------------------*/

#pragma once
#include <vector>
#include <string>

class Directions
{
private:
	char _commandType;
	int _degree;
	int _scale;
	int _forwardThrust;
	int _rotationDir;

public:
	Directions()
	{
		ClearDirections();
	}

	void SetType(char aType);
	void SetDegrees(int aDeg);
	void SetScale(int aScale);
	void SetForThrust(int aThrust);
	void SetRotationDir(int _aDir);
	char GetType();
	int GetDegrees();
	int GetScale();
	int GetForThrust();
	int GetRotationDir();


	//Sets all fields to 0
	void ClearDirections();

	//Returns string of instructions
	std::string GetInstructions();
};