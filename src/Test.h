/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include "Calculator.h"

#define ANGLE_THRESHOLD	1E-7

class Test
{
public:
	static bool Check();

	template<typename T>
	static bool angleTest(const T &_vector1, const T &_vector2, const double _angle)
	{
		return fabs(Calculator::angle<T>(_vector1, _vector2) - _angle) < ANGLE_THRESHOLD;
	}

	template<typename T>
	static bool angleSignedTest(const T &_vector1, const T &_vector2, const T &_normal, const double _angle)
	{
		return fabs(Calculator::signedAngle<T>(_vector1, _vector2, _normal) - _angle) < ANGLE_THRESHOLD;
	}

private:
	Test();
	~Test();
};
