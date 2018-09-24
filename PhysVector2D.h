/*	------------------------------------
	PHYS VECTOR 2D
	------------------------------------
	Description:
		Phys Vector objects hold XY positions, and are used to calculate directional vectors on a 2D plane (in this case, the screen's X/Y planes).
	------------------------------------
	Fields:
		x (float)
		y (float)

	Functions:
		public float GetX();
		public float GetY();

		public void Zero();
		public bool IsZero();

		public float Length();
		public float LengthSq();

		public void Normalise();
		public PhysVector2D GetNormalised();

		public float Dot(PhysVector2D rhs);
		public int Sign(PhysVector2D rhs);
		public PhysVector2D Perp();

		public void Truncate(float maxLength);
		public float Distance(PhysVector2D v2);
		public float DistanceSq(PhysVector2D v2);
		public void Reflect(PhysVector2D norm);

		public PhysVector2D GetReverse();
		public PhysVector2D Copy();

	Operator Overrides:
		bool operator==(PhysVector2D rhs);
		bool operator!=(PhysVector2D rhs);
		PhysVector2D operator+(PhysVector2D rhs);
		PhysVector2D operator-();
		PhysVector2D operator-(PhysVector2D rhs);
		PhysVector2D operator*(PhysVector2D rhs);
		PhysVector2D operator*(float rhs);
		PhysVector2D operator/(PhysVector2D rhs);

	------------------------------------*/

#pragma once
#include <math.h>

class PhysVector2D
{
private:
	float x;
	float y;

public:
	PhysVector2D();
	PhysVector2D(float X, float Y);

	float GetX();
	float GetY();

	void Zero();
	bool IsZero();

	float Length();
	float LengthSq();

	void Normalise();
	PhysVector2D GetNormalised();

	float Dot(PhysVector2D rhs);
	int Sign(PhysVector2D rhs);
	PhysVector2D Perp();

	void Truncate(float maxLength);
	float Distance(PhysVector2D v2);
	float DistanceSq(PhysVector2D v2);
	void Reflect(PhysVector2D norm);

	PhysVector2D GetReverse();
	PhysVector2D Copy();

	//OPERATOR OVERRIDES
	bool operator==(PhysVector2D rhs);
	bool operator!=(PhysVector2D rhs);
	PhysVector2D operator+(PhysVector2D rhs);
	PhysVector2D operator-();
	PhysVector2D operator-(PhysVector2D rhs);
	PhysVector2D operator*(PhysVector2D rhs);
	PhysVector2D operator*(float rhs);
	PhysVector2D operator/(PhysVector2D rhs);
};