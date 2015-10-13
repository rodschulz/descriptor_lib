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

	double distance(const cv::Mat &_vector1, const cv::Mat &_vector2) const;
	cv::Mat calculateCenters(const int _clusterNumber, const cv::Mat &_items, const cv::Mat &_labels) const;
	cv::Mat calculateCenters(const int _clusterNumber, const cv::Mat &_items, const cv::Mat &_labels, std::vector<int> &_itemsPerCenter) const;

private:
	int permutationSize;

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

	Permutation getClosestPermutation(const cv::Mat &_vector1, const cv::Mat &_vector2) const;
};
