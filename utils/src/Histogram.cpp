/**
 * Author: rodrigo
 * 2015
 */
#include "Histogram.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <cmath>
#include <pcl/pcl_macros.h>


std::ostream& operator<<(std::ostream &_stream, const Bins &_bins)
{
	_stream.precision(2);

	double step = _bins.dimension == ANGLE ? RAD2DEG(_bins.step) : _bins.step;
	for (size_t i = 0; i < _bins.bins.size(); i++)
	{
		_stream << std::fixed << "[" << step * i << "]:";
		_stream << _bins.bins[i];
		if (i != _bins.bins.size() - 1)
			_stream << "\t";
	}
	_stream << "\n";
	return _stream;
}

void printBins(const Bins &_data)
{
	for (size_t i = 0; i < _data.bins.size(); i++)
	{
		printf("[%1.2f]:%1.2f", _data.step * i, _data.bins[i]);
		if (i != _data.bins.size() - 1)
			printf(" - ");
	}
	printf("\n");
}


Histogram::Histogram(const Dimension _dimension)
{
	minData = maxData = 0;
	dimension = _dimension;
}

void Histogram::add(const double _element)
{
	data.push_back(_element);

	minData = minData > _element ? _element : minData;
	maxData = maxData < _element ? _element : maxData;
}

Bins Histogram::getBins(const double binSize_,
						const double lowerBound_,
						const double upperBound_) const
{
	int binNumber = ceil((upperBound_ - lowerBound_) / binSize_);
	// binNumber = binNumber % 2 > 0 ? binNumber : binNumber + 1;

	Bins b;
	b.bins.resize(binNumber, 0);
	b.step = binSize_;
	b.dimension = dimension;

	// Put elements in bins
	if (!data.empty())
	{
		int outElements = 0;
		for (size_t i = 0; i < data.size(); i++)
		{
			int index = ((data[i] - lowerBound_) / binSize_);
			if (index >= binNumber)
				outElements++;
			else
				b.bins[index]++;
		}

		// Normalize
		int size = data.size() - outElements;
		for (int i = 0; i < binNumber && size > 0; i++)
			b.bins[i] /= size;
	}

	return b;
}

Bins Histogram::getBins(const double binSize_) const
{
	return getBins(binSize_, minData, maxData);
}
