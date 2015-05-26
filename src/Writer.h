/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <vector>
#include <string>
#include "Hist.h"

using namespace std;

#define OUTPUT_FOLDER		"./output/"

class Writer
{
public:
	static void writeData(const string &_filename, const vector<double> &_curvatures, const vector<Hist> &_curvatureHistograms, const vector<Hist> &_angleHistograms);
	static void writeHistogram(const string &_filename, const string &_histogramTitle, const vector<Hist> &_histograms, const int _binsNumber, const double _lowerBound = -1, const double _upperBound = -1);

private:
	Writer();
	~Writer();

	static void generateScript(const string &_filename, const string &_histogramTitle, const int _bandsNumber);
};
