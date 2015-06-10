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
	static void calculateAngleHistograms(const vector<BandPtr> &_bands, vector<Hist> &_histograms);

	static void calculateSequences(const vector<BandPtr> &_bands, const double _binSize, const double _sequenceStep);

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

private:
	Calculator();
	~Calculator();
};

