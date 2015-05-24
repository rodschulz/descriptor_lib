/**
 * Author: rodrigo
 * 2015
 */
#include "Hist.h"
#include <stdlib.h>
#include <stdio.h>

#define RAD2DEG(x)	((x) * 57.29578)

Hist::Hist(const Dimension _dimension)
{
	minData = maxData = 0;
	dimension = _dimension;
}

Hist::~Hist()
{
}

ostream& operator<<(ostream &_stream, const Bins &_bins)
{
	_stream.precision(2);

	double step = _bins.dimension == ANGLE ? RAD2DEG(_bins.step) : _bins.step;
	for (size_t i = 0; i < _bins.bins.size(); i++)
	{
		_stream << fixed << "[" << step * i << "]:";
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

void Hist::getBins(const int _binsNumber, const double _lowerBound, const double _upperBound, Bins &_bins)
{
	double step = (_upperBound - _lowerBound) / _binsNumber;
	vector<double> binsVector(_binsNumber, 0);

	for (size_t i = 0; i < data.size(); i++)
	{
		int index = data[i] / step;
		binsVector[index]++;
	}
	for (size_t i = 0; i < binsVector.size(); i++)
		binsVector[i] /= data.size();

	_bins.bins = binsVector;
	_bins.step = step;
	_bins.dimension = dimension;
}

void Hist::getBins(const int _binsNumber, Bins &_bins)
{
	getBins(_binsNumber, minData, maxData, _bins);
}
