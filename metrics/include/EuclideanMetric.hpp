/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <stdexcept>
#include "Metric.hpp"

class EuclideanMetric: public Metric
{
public:
	// Constructor
	EuclideanMetric()
	{
	}

	// Destructor
	~EuclideanMetric()
	{
	}

	// Returns the distance between the two given vectors according to this metric
	inline double distance(const cv::Mat &_vector1, const cv::Mat &_vector2) const
	{
		if (_vector1.cols != _vector2.cols || _vector1.rows != _vector2.rows)
			throw std::runtime_error("Invalid matrix dimensions in distance");

		return cv::norm(_vector1, _vector2);
	}

	// Calculates the center of the given set of items, according to this metric
	inline cv::Mat calculateCenters(const int _clusterNumber, const cv::Mat &_items, const cv::Mat &_labels, std::vector<int> &_itemsPerCenter) const
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

		for (int i = 0; i < newCenters.rows; i++)
			newCenters.row(i) /= _itemsPerCenter[i];

		return newCenters;
	}

	// Returns the type of the current metric
	MetricType getType() const
	{
		return METRIC_EUCLIDEAN;
	}
};
