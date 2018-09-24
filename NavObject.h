//Placeholder class for actual object class - originates from full project!//
#pragma once
class NavObject
{
private:
	int _x;
	int _y;
	int _length;
	int _width;
	short _urgency;
	bool _attract;
	bool _dynamic;
	double _xVel;
	double _yVel;

	NavObject(){}
public:
	NavObject(int X, int Y, int Width, int Length, short Urgency, bool Attract)
	{
		_x = X;
		_y = Y;
		_width = Width;
		_length = Length;
		_urgency = Urgency;
		_attract = Attract;
		_dynamic = false;
		_xVel = 0.0;
		_yVel = 0.0;
	}

	NavObject(int X, int Y, int Width, int Length, short Urgency, bool Attract, bool Dynamic, double XVel, double YVel)
	{
		_x = X;
		_y = Y;
		_width = Width;
		_length = Length;
		_urgency = Urgency;
		_attract = Attract;
		_dynamic = Dynamic;
		_xVel = XVel;
		_yVel = YVel;
	}

	int GetX()
	{
		return _x;
	}

	int GetY()
	{
		return _y;
	}

	int GetWidth()
	{
		return _width;
	}

	int GetLength()
	{
		return _length;
	}

	short GetUrgency()
	{
		return _urgency;
	}

	bool GetAttract()
	{
		return _attract;
	}

	bool GetDynamic()
	{
		return _dynamic;
	}

	double GetXVel()
	{
		return _xVel;
	}

	double GetYVel()
	{
		return _yVel;
	}
};