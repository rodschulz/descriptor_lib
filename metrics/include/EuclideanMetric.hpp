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
	EuclideanMetric() {}
	~EuclideanMetric() {}

	/**************************************************/
	inline double distance(const cv::Mat &vector1_,
						   const cv::Mat &vector2_) const
	{
		if (vector1_.cols != vector2_.cols || vector1_.rows != vector2_.rows)
			throw std::runtime_error("Invalid matrix dimensions in distance");

		return cv::norm(vector1_, vector2_);
	}

	/**************************************************/
	inline cv::Mat calculateMeans(const int clusterNumber_,
								  const cv::Mat &items_,
								  const cv::Mat &labels_,
								  const cv::Mat &currentMeans_ = cv::Mat()) const
	{
		std::vector<int> itemCount = std::vector<int>(clusterNumber_, 0);
		cv::Mat newCenters = cv::Mat::zeros(clusterNumber_, items_.cols, CV_32FC1);

		// Iterate over labels calculating the
		for (int i = 0; i < labels_.rows; i++)
		{
			int clusterIndex = labels_.at<int>(i);

			newCenters.row(clusterIndex) += items_.row(i);
			itemCount[clusterIndex] += 1;
		}

		for (int i = 0; i < newCenters.rows; i++)
			newCenters.row(i) /= itemCount[i];

		return newCenters;
	}

	/**************************************************/
	inline MetricType getType() const
	{
		return METRIC_EUCLIDEAN;
	}

	/**************************************************/
	inline std::vector<std::string> getConstructionParams() const
	{
		return std::vector<std::string>();
	}

	/**************************************************/
	inline void validateMeans(cv::Mat &means_) const
	{
	}
};
