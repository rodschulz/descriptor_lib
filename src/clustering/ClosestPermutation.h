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

	double distance(const cv::Mat &_vector1, const cv::Mat &_vector2);
	cv::Mat calculateCenters(const int _clusterNumber, const cv::Mat &_items, const cv::Mat &_labels);
	cv::Mat calculateCenters(const int _clusterNumber, const cv::Mat &_items, const cv::Mat &_labels, std::vector<int> &_itemsPerCenter);

private:
	int permutationSize;
	cv::Mat cachedVector;
	cv::Mat cachedPermutation;

	cv::Mat getClosestPermutation(const cv::Mat &_vector1, const cv::Mat &_vector2);
};
