/**
 * Author: rodrigo
 * 2015
 */
#include "ClosestPermutationMetric.hpp"
#include <opencv2/core/types_c.h>

// TODO check if this implementation can be improved in terms of speed
// TODO check what is done now and check if the opencv's kmeans can be used instead, with different versions of the vectors (for the permutations)
// TODO try to do something to prevent clusters NaN begin generated (I think those are because empty clusters are being created) => check the average calculation

ClosestPermutationMetric::ClosestPermutationMetric(const int _permutationSize)
{
	permutationSize = _permutationSize;
}

ClosestPermutationMetric::~ClosestPermutationMetric()
{
}

double ClosestPermutationMetric::distance(const cv::Mat &_vector1, const cv::Mat &_vector2) const
{
	return getClosestPermutation(_vector1, _vector2).distance;
}

cv::Mat ClosestPermutationMetric::calculateCenters(const int _clusterNumber, const cv::Mat &_items, const cv::Mat &_labels, std::vector<int> &_itemsPerCenter) const
{
	_itemsPerCenter = std::vector<int>(_clusterNumber, 0);

	// Matrixes holding the first vector used to compare and the new centers
	cv::Mat firstVector = cv::Mat::zeros(_clusterNumber, _items.cols, CV_32FC1);
	cv::Mat newCenters = cv::Mat::zeros(_clusterNumber, _items.cols, CV_32FC1);

	// Iterate over labels calculating the
	std::vector<bool> begin(_clusterNumber, true);
	for (int i = 0; i < _labels.rows; i++)
	{
		int clusterIndex = _labels.at<int>(i);

		// Track if every centroid has got its first element
		if (begin[clusterIndex])
		{
			_items.row(i).copyTo(firstVector.row(clusterIndex));
			_items.row(i).copyTo(newCenters.row(clusterIndex));
			begin[clusterIndex] = false;
		}
		else
		{
			Permutation closestPermutation = getClosestPermutation(firstVector.row(clusterIndex), _items.row(i));

			// Add the row's value to new center
			newCenters.row(clusterIndex).colRange(0, newCenters.cols - (closestPermutation.index * permutationSize)) += _items.row(i).colRange(closestPermutation.index * permutationSize, newCenters.cols);
			newCenters.row(clusterIndex).colRange(newCenters.cols - (closestPermutation.index * permutationSize), newCenters.cols) += _items.row(i).colRange(0, closestPermutation.index * permutationSize);
		}

		_itemsPerCenter[clusterIndex] += 1;
	}

	for (int i = 0; i < newCenters.rows; i++)
		newCenters.row(i) /= _itemsPerCenter[i];

	return newCenters;
}
