/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <opencv2/core/core.hpp>

class Metric
{
public:
	Metric()
	{
	}
	virtual ~Metric()
	{
	}

	virtual double distance(const cv::Mat _vector1, const cv::Mat _vector2) = 0;
	virtual cv::Mat average(const cv::Mat &_vectors) = 0;
};
