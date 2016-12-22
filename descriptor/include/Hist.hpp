/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <vector>
#include <ostream>

enum Dimension
{
	ANGLE, OTHER
};

struct Bins
{
	std::vector<float> bins;
	double step;
	Dimension dimension;

	Bins()
	{
		bins.clear();
		step = 0;
		dimension = OTHER;
	}

	Bins(const Bins &other_)
	{
		bins = other_.bins;
		step = other_.step;
		dimension = other_.dimension;
	}
};

class Hist
{
public:
	Hist(const Dimension _dimension = OTHER);
	~Hist() {};

	/**************************************************/
	void add(const double _element);

	/**************************************************/
	Bins getBins(const double binSize_,
				 const double lowerBound_,
				 const double upperBound_) const;

	/**************************************************/
	Bins getBins(const double binSize_) const;

private:
	std::vector<double> data;
	double minData;
	double maxData;
	Dimension dimension;
};

std::ostream& operator<<(std::ostream &_stream, const Bins &_bins);
void printBins(const Bins &_data);
