/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include "Metric.hpp"

class ClosestPermutationMetric: public Metric
{
public:
	// Constructor
	ClosestPermutationMetric(const int _permutationSize, const bool _useConfidence);

	// Destructor
	~ClosestPermutationMetric();

	// Returns the distance between the given vectors, according to the current metric
	double distance(const cv::Mat &_vector1, const cv::Mat &_vector2) const;

	// Returns the central point amongst the given items, according to the given labels
	cv::Mat calculateCenters(const int _clusterNumber, const cv::Mat &_items, const cv::Mat &_labels, std::vector<int> &_itemsPerCenter) const;

private:

	int permutationSize; // Size of the permutation used by this metric (number of elements moved for each permutation)
	bool useConfidence;

	struct Permutation
	{
		double distance;
		double confidence;
		int index;

		Permutation()
		{
			distance = confidence = index = -1;
		}

		Permutation(const double _distance, double _confidence, int _index)
		{
			distance = _distance;
			confidence = _confidence;
			index = _index;
		}
	};

	inline Permutation getClosestPermutation(const cv::Mat &_vector1, const cv::Mat &_vector2) const;
};
