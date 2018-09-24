#include "PhysVector2D.h"
/*-------------------------------
NOTE: The functions below are standard for 2D Vector calculations; Unless you have extremely
good reason to, you should NOT change these functions from their original state!
-------------------------------*/


//default constructor; sets initial values to 0
PhysVector2D::PhysVector2D()
{
	x = 0;
	y = 0;
};

//constructor. Sets x and Y to inputted parameters
PhysVector2D::PhysVector2D(float X, float Y)
{
	x = X;
	y = Y;
};

//returns x field value
float PhysVector2D::GetX()
{
	return x;
};

//returns y field value
float PhysVector2D::GetY()
{
	return y;
};

//set X, Y, to 0
void PhysVector2D::Zero()
{
	x = 0.0;
	y = 0.0;
};

//return true if all values are 0
bool PhysVector2D::IsZero()
{
	if (x == 0.0 && y == 0.0)
		return true;
	else
		return false;
};

//return length of vector
float PhysVector2D::Length()
{
	return sqrt((x * x) + (y * y));
};

//return squared length of vector
float PhysVector2D::LengthSq()
{
	return (x * x) + (y * y);
};

//normalise self to a unit vector of length 1.0
void PhysVector2D::Normalise()
{
	float l = sqrt((x*x) + (y*y));

	if (l == 0.0)
	{
		x = 0.0;
		y = 0.0;
	}
	else
	{
		x = x / l;
		y = y / l;
	}
};

//return a normalised copy of self
PhysVector2D PhysVector2D::GetNormalised()
{
	PhysVector2D result = PhysVector2D(x, y);
	result.Normalise();
	return result;
};

//return the dot (inner) product of self and rhs vector
float PhysVector2D::Dot(PhysVector2D rhs)
{
	return (x * rhs.GetX()) + (y * rhs.GetY());
};

//return +1 if rhs is clockwise of self, or -1 if counter-clockwise of self
int PhysVector2D::Sign(PhysVector2D rhs)
{
	//assumes Y axis points down and X points right
	if ((y * rhs.GetX()) > (y * rhs.GetY()))
		return -1;
	else
		return 1;
};

//return vector perpendicular to self
PhysVector2D PhysVector2D::Perp()
{
	return PhysVector2D(-y, x);
};

//limit length (scale X and Y) to maxlength
void PhysVector2D::Truncate(float maxLength)
{
	if (Length() > maxLength)
	{
		Normalise();
		x *= maxLength;
		y *= maxLength;
	}
};

//distance between self and v2
float PhysVector2D::Distance(PhysVector2D v2)
{
	float dx = v2.GetX() - x;
	float dy = v2.GetY() - y;
	return sqrt((dx*dx) + (dy * dy));
};

//squared distance between self and v2
float PhysVector2D::DistanceSq(PhysVector2D v2)
{
	float dx = v2.GetX() - x;
	float dy = v2.GetY() - y;
	return (dx*dx) + (dy * dy);
};

//reflect self around the norm vector provided
void PhysVector2D::Reflect(PhysVector2D norm)
{
	PhysVector2D result = PhysVector2D(x, y);
	result = result + (norm.GetReverse() * Dot(norm));

	x = result.GetX();
	y = result.GetY();
};

//return new vector that is reverse of self
PhysVector2D PhysVector2D::GetReverse()
{
		
	return -PhysVector2D(x, y);
};

//returns a new copy of the vector
PhysVector2D PhysVector2D::Copy()
{
	return PhysVector2D(x, y);
};

//OPERATOR OVERRIDES

//returns true if called PhysVector2D has the same values as the compared PhysVector2D
bool PhysVector2D::operator==(PhysVector2D rhs)
{
	return (x == rhs.GetX() && y == rhs.GetY());
};

//returns true if called PhysVector2D does not have the same values as the compared PhysVector2D
bool PhysVector2D::operator!=(PhysVector2D rhs)
{
	return (x != rhs.GetX() || y != rhs.GetY());
};

//Adds the X and Y values of the right PhysVector2D to the left PhysVector2D
PhysVector2D PhysVector2D::operator+(PhysVector2D rhs)
{
	return PhysVector2D((x + rhs.GetX()), (y + rhs.GetY()));
};

//inverts PhysVector2D values
PhysVector2D PhysVector2D::operator-()
{
	return PhysVector2D((x * -1), (y * -1));
};

//Subtracts the X and Y values of the right PhysVector2D to the left PhysVector2D
PhysVector2D PhysVector2D::operator-(PhysVector2D rhs)
{
	return PhysVector2D((x - rhs.GetX()), (y - rhs.GetY()));
};

//multiplies the X and Y values of the right PhysVector2D with the left PhysVector2D
PhysVector2D PhysVector2D::operator*(PhysVector2D rhs)
{
	return PhysVector2D((x * rhs.GetX()),(y * rhs.GetY()));
};

//multiplies the PhysVector2D's X Y values with the inputted float value
PhysVector2D PhysVector2D::operator*(float rhs)
{
	return PhysVector2D((x * rhs), (y * rhs));
};

//Divides the X and Y values of the left PhysVector2D by the right PhysVector2D
PhysVector2D PhysVector2D::operator/(PhysVector2D rhs)
{
	return PhysVector2D((x * rhs.GetX()), (y * rhs.GetY()));
};