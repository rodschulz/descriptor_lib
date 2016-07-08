/**
 * Author: rodrigo
 * 2016     
 */
#pragma once

#include "ClosestPermutationMetric.hpp"

class ClosestPermutationWithConfidenceMetric: public ClosestPermutationMetric
{
public:
	// Constructor
	ClosestPermutationWithConfidenceMetric(const int permutationSize_) :
			ClosestPermutationMetric(permutationSize_)
	{
	}

	// Destructor
	~ClosestPermutationWithConfidenceMetric()
	{
	}

	// Returns the distance between the given vectors, according to the current metric
	inline double distance(const cv::Mat &_vector1, const cv::Mat &_vector2) const
	{
		throw std::runtime_error("Metric not yet implemented");
	}

	// Returns the central point amongst the given items, according to the given labels
	inline cv::Mat calculateMeans(const int clusterNumber_, const cv::Mat &items_, const cv::Mat &labels_, const cv::Mat &currentMeans_ = cv::Mat()) const
	{
		throw std::runtime_error("Metric not yet implemented");
	}

	inline Permutation getClosestPermutation(const cv::Mat &_vector1, const cv::Mat &_vector2) const
	{
		throw std::runtime_error("Metric not yet implemented");
	}

	// Returns the type of the current metric
	MetricType getType() const
	{
		return METRIC_CLOSEST_PERMUTATION_WITH_CONFIDENCE;
	}

	// Returns the parameters used to construct the current instance
	std::vector<std::string> getConstructionParams() const
	{
		std::vector<std::string> params;
		params.push_back(boost::lexical_cast<std::string>(permutationSize));
		return params;
	}

	// Validates and fixes the given means, according to the metric's definition
	void validateMeans(cv::Mat &means_) const
	{
	}
};
