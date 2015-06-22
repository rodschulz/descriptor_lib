/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include "Extractor.h"
#include "Hist.h"
#include "Helper.h"
#include <vector>

using namespace std;
using namespace pcl;

class Calculator
{
public:
	static void calculateAngleHistograms(const vector<BandPtr> &_bands, vector<Hist> &_histograms, const bool _useProjection);

	static void calculateSequences(const vector<BandPtr> &_bands, const double _binSize, const double _sequenceStep, const bool _useProjection);

	template<class T>
	static inline double angle(const T &_vector1, const T &_vector2)
	{
		return atan2(_vector1.cross(_vector2).norm(), _vector1.dot(_vector2));
	}

	template<class T>
	static inline double signedAngle(const T &_vector1, const T &_vector2, const T &_normal)
	{
		double direction = _normal.dot(_vector1.cross(_vector2));

		// Check if the cross product is not zero
		if (fabs(direction) > 1E-7)
		{
			if (_normal.dot(_vector1.cross(_vector2)) < 0)
				return -atan2(_vector1.cross(_vector2).norm(), _vector1.dot(_vector2));
			else
				return atan2(_vector1.cross(_vector2).norm(), _vector1.dot(_vector2));
		}
		else
		{
			if (_vector1.dot(_vector2) >= 0)
				return 0;
			else
				return M_PI;

		}
	}

	static inline char getSequenceChar(const double _value, const double _step)
	{
		int index = _value / _step;
		return 'a' + index;
	}

	static inline double calculateAngle(const Vector3f &_targetNormal, const Vector3f &_normal, const BandPtr &_band, const bool _useProjection)
	{
		Vector3f pointNormal;
		if (_useProjection)
			pointNormal = _band->plane.projection(_normal).normalized();
		else
			pointNormal = _normal;

		return angle<Vector3f>(_targetNormal, pointNormal);
	}

private:
	Calculator();
	~Calculator();
};

