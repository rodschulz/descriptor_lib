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
	static void writeHistogram(const string &_filename, const string &_histogramTitle, const vector<Hist> &_histograms, const double _binSize, const double _lowerBound = -1, const double _upperBound = -1);

private:
	Writer();
	~Writer();

	static void generateScript(const string &_filename, const string &_histogramTitle, const int _bandsNumber, const double _binSize, const double _lowerLimit, const double _upperLimit);
};
