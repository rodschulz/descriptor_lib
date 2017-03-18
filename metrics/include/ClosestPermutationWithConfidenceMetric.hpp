/**
 * Author: rodrigo
 * 2016
 */
#pragma once

#include "ClosestPermutationMetric.hpp"


class ClosestPermutationWithConfidenceMetric: public ClosestPermutationMetric
{
public:
	ClosestPermutationWithConfidenceMetric(const int permutationSize_) :
		ClosestPermutationMetric(permutationSize_) {}
	~ClosestPermutationWithConfidenceMetric() {}

	/**************************************************/
	inline double distance(const cv::Mat &vector1_,
						   const cv::Mat &vector2_) const
	{
		throw std::runtime_error("Metric not yet implemented");
	}

	/**************************************************/
	inline cv::Mat calculateMeans(const int clusterNumber_,
								  const cv::Mat &items_,
								  const cv::Mat &labels_,
								  const cv::Mat &currentMeans_ = cv::Mat()) const
	{
		throw std::runtime_error("Metric not yet implemented");
	}

	/**************************************************/
	inline Permutation getClosestPermutation(const cv::Mat &vector1_,
			const cv::Mat &vector2_) const
	{
		throw std::runtime_error("Metric not yet implemented");
	}

	/**************************************************/
	MetricType getType() const
	{
		return METRIC_CLOSEST_PERMUTATION_WITH_CONFIDENCE;
	}
};
