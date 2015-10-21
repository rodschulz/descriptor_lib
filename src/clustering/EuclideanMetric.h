/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include "Metric.h"

class EuclideanMetric: public Metric
{
public:
	EuclideanMetric()
	{
	}
	~EuclideanMetric()
	{
	}

	double distance(const cv::Mat &_vector1, const cv::Mat &_vector2) const
	{
		return cv::norm(_vector1, _vector2);
	}

	cv::Mat calculateCenters(const int _clusterNumber, const cv::Mat &_items, const cv::Mat &_labels, std::vector<int> &_itemsPerCenter) const
	{
		_itemsPerCenter = std::vector<int>(_clusterNumber, 0);
		cv::Mat newCenters = cv::Mat::zeros(_clusterNumber, _items.cols, CV_32FC1);

		// Iterate over labels calculating the
		for (int i = 0; i < _labels.rows; i++)
		{
			int clusterIndex = _labels.at<int>(i);

			newCenters.row(clusterIndex) += _items.row(i);
			_itemsPerCenter[clusterIndex] += 1;
		}

		std::cout << "+++\n"<< newCenters << std::endl;
		for (int i = 0; i < newCenters.rows; i++)
			newCenters.row(i) /= _itemsPerCenter[i];
		std::cout << "%%%\n"<< newCenters << std::endl;

		return newCenters;
	}
};
