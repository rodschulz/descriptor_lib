/**
 * Author: rodrigo
 * 2015
 */
#include "Hist.h"
#include <stdlib.h>
#include <stdio.h>
#include  <cmath>

#define RAD2DEG(x)	((x) * 57.29578)
#define DEG2RAD(x)	((x) * 0.017453293)

Hist::Hist(const Dimension _dimension)
{
	minData = maxData = 0;
	dimension = _dimension;
}

Hist::~Hist()
{
}

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

void Hist::add(const double _element)
{
	data.push_back(_element);

	minData = minData > _element ? _element : minData;
	maxData = maxData < _element ? _element : maxData;
}

void Hist::getBins(const double _binSize, const double _lowerBound, const double _upperBound, Bins &_bins) const
{
	int binNumber = ceil((_upperBound - _lowerBound) / _binSize);
	binNumber = binNumber % 2 > 0 ? binNumber : binNumber + 1;

	_bins.bins.clear();
	_bins.bins.resize(binNumber, 0);
	_bins.step = _binSize;
	_bins.dimension = dimension;

	for (size_t i = 0; i < data.size(); i++)
	{
		int index = ((data[i] - _lowerBound) / _binSize);
		index = index >= binNumber ? binNumber - 1 : index;

		_bins.bins[index]++;
	}
	for (int i = 0; i < binNumber; i++)
		_bins.bins[i] /= data.size();
}

void Hist::getBins(const double _binSize, Bins &_bins) const
{
	getBins(_binSize, minData, maxData, _bins);
}
