/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include "Extractor.h"
#include "Hist.h"
#include <vector>

using namespace std;
using namespace pcl;

class Calculator
{
public:
	static void calculateMeanCurvature(const vector<BandPtr> &_bands, vector<double> &_curvatures);

	static void calculateCurvatureHistograms(const vector<BandPtr> &_bands, vector<Hist> &_histograms);
	static void calculateAngleHistograms(const vector<BandPtr> &_bands, vector<Hist> &_histograms, const bool _useProjection);

	static void calculateSequences(const vector<BandPtr> &_bands, const double _binSize, const double _sequenceStep, const bool _useProjection);

	template<class T>
	static inline double angleBetween(const T &_vector1, const T &_vector2)
	{
		return atan2(_vector1.cross(_vector2).norm(), _vector1.dot(_vector2));
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

		return angleBetween<Vector3f>(_targetNormal, pointNormal);
	}

private:
	Calculator();
	~Calculator();
};

