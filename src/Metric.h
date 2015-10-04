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

	virtual double distance(const cv::Mat &_vector1, const cv::Mat &_vector2) = 0;
	virtual cv::Mat calculateCenters(const int _clusterNumber, const cv::Mat &_vectors, const cv::Mat &_labels) = 0;
	virtual cv::Mat calculateCenters(const int _clusterNumber, const cv::Mat &_vectors, const cv::Mat &_labels, std::vector<int> &_itemsPerCenter) = 0;
};
