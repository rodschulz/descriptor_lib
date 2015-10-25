/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <opencv2/core/core.hpp>
#include <boost/shared_ptr.hpp>

// Forward declaration to define a metric's shared pointer
class Metric;
typedef boost::shared_ptr<Metric> MetricPtr;

class Metric
{
public:
	Metric()
	{
	}

	virtual ~Metric()
	{
	}

	// Methods to implement
	virtual double distance(const cv::Mat &_vector1, const cv::Mat &_vector2) const = 0;
	virtual cv::Mat calculateCenters(const int _clusterNumber, const cv::Mat &_items, const cv::Mat &_labels, std::vector<int> &_itemsPerCenter) const = 0;

	// Auxiliary method to ease calls
	cv::Mat calculateCenters(const int _clusterNumber, const cv::Mat &_items, const cv::Mat &_labels) const
	{
		std::vector<int> itemCount;
		return calculateCenters(_clusterNumber, _items, _labels, itemCount);
	}
};
