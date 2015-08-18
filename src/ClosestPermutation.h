/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include "Metric.h"

class ClosestPermutation: public Metric
{
public:
	ClosestPermutation(const int _permutationSize);
	~ClosestPermutation();

	double distance(const cv::Mat _vector1, const cv::Mat _vector2);
	cv::Mat average(const cv::Mat &_vectors);

private:
	int permutationSize;
};
