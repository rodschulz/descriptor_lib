/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <vector>
#include <ostream>

using namespace std;


struct Bins
{
	vector <double> bins;
	double step;
	
	Bins()
	{
		bins.clear();
		step = 0;
	}
};

class Hist
{
public:
	Hist();
	Hist(const vector <double> &_data);
	~Hist();
	
	void add(const double _element);
	void getBins(const int _binsNumber, const double _lowerBound, const double _upperBound, Bins &_bins);
	void getBins(const int _binsNumber, Bins &_bins);
	
private:
	vector <double> data;
	double minData;
	double maxData;
};

void printBins(const Bins &_data);