/**
 * Author: rodrigo
 * 2016     
 */
#include "ClosestPermutationWithConfidenceMetric.hpp"
#include <iostream>
#include <stdexcept>
#include <opencv2/core/types_c.h>

ClosestPermutationWithConfidenceMetric::ClosestPermutationWithConfidenceMetric(const int _permutationSize)
{
	permutationSize = _permutationSize;
}

ClosestPermutationWithConfidenceMetric::~ClosestPermutationWithConfidenceMetric()
{
}

double ClosestPermutationWithConfidenceMetric::distance(const cv::Mat &_vector1, const cv::Mat &_vector2) const
{
	throw std::runtime_error("Metric not yet implemented");

	return getClosestPermutation(_vector1, _vector2).distance;
}

cv::Mat ClosestPermutationWithConfidenceMetric::calculateCenters(const int _clusterNumber, const cv::Mat &_items, const cv::Mat &_labels, std::vector<int> &_itemsPerCenter) const
{
	throw std::runtime_error("Metric not yet implemented");

	_itemsPerCenter = std::vector<int>(_clusterNumber, 0);

	// Matrixes holding the first vector used to compare and the new centers
	cv::Mat firstVector = cv::Mat::zeros(_clusterNumber, _items.cols, CV_32FC1);
	cv::Mat newCenters = cv::Mat::zeros(_clusterNumber, _items.cols, CV_32FC1);

//	// Iterate over labels calculating the
//	std::vector<bool> begin(_clusterNumber, true);
//	for (int i = 0; i < _labels.rows; i++)
//	{
//		int clusterIndex = _labels.at<int>(i);
//
//		// Track if every centroid has got its first element
//		if (begin[clusterIndex])
//		{
//			_items.row(i).copyTo(firstVector.row(clusterIndex));
//			_items.row(i).copyTo(newCenters.row(clusterIndex));
//			begin[clusterIndex] = false;
//		}
//		else
//		{
//			Permutation closestPermutation = getClosestPermutation(firstVector.row(clusterIndex), _items.row(i));
//
//			// Add the row's value to new center
//			newCenters.row(clusterIndex).colRange(0, newCenters.cols - (closestPermutation.index * permutationSize)) += _items.row(i).colRange(closestPermutation.index * permutationSize, newCenters.cols);
//			newCenters.row(clusterIndex).colRange(newCenters.cols - (closestPermutation.index * permutationSize), newCenters.cols) += _items.row(i).colRange(0, closestPermutation.index * permutationSize);
//		}
//
//		_itemsPerCenter[clusterIndex] += 1;
//	}
//
//	for (int i = 0; i < newCenters.rows; i++)
//		newCenters.row(i) /= _itemsPerCenter[i];

	return newCenters;
}

ClosestPermutationWithConfidenceMetric::Permutation ClosestPermutationWithConfidenceMetric::getClosestPermutation(const cv::Mat &_vector1, const cv::Mat &_vector2) const
{
	if (_vector1.cols != _vector2.cols || _vector1.rows != _vector2.rows)
		throw std::runtime_error("Invalid matrix dimensions in distance");
	if (_vector1.cols % permutationSize != 0 && _vector2.cols % permutationSize != 0)
		throw std::runtime_error("Permutation size incompatible with vector lenghts");

	// Number of permutations to be evaluated
	int permutationNumber = std::floor(_vector1.cols / permutationSize);

	// Auxiliar vectors
	std::vector<double> bandConfidence(permutationNumber, 1);
	std::vector<double> bandDistance(permutationNumber, 0);

	// Results
	double minDistance = std::numeric_limits<double>::max();
	double minConfidence = -1;
	int minIndex = -1;

//	// Iterate over every possible permutation
//	double subtract = 1.0 / permutationSize;
//	for (int i = 0; i < permutationNumber; i++)
//	{
//		int baseIndex = i * permutationSize;
//		for (int j = 0; j < _vector1.cols; j++)
//		{
//			float x1 = _vector1.at<float>(j);
//			float x2 = _vector2.at<float>((baseIndex + j) % _vector1.cols);
//
//			int bandIndex = j / permutationSize;
//			if (useConfidence && (x1 > M_PI || x2 > M_PI))
//				bandConfidence[bandIndex] -= subtract;
//			else
//				bandDistance[bandIndex] += (x1 - x2) * (x1 - x2);
//		}
//
//		// Calculate the actual distance between vectors
//		double distance = 0;
//		double confidenceSum = 0;
//		if (useConfidence)
//		{
//			for (int j = 0; j < permutationNumber; j++)
//				confidenceSum += bandConfidence[j];
//			if (confidenceSum > 0)
//			{
//				for (int j = 0; j < permutationNumber; j++)
//					distance += (bandDistance[j] * bandConfidence[j] / confidenceSum);
//			}
//			else
//				distance = std::numeric_limits<double>::max();
//		}
//		else
//		{
//			for (size_t j = 0; j < bandDistance.size(); j++)
//				distance += bandDistance[j];
//			distance = sqrt(distance);
//		}
//
//		// Check if the permutation is closer
//		if (distance < minDistance)
//		{
//			minDistance = distance;
//			minConfidence = confidenceSum / permutationNumber;
//			minIndex = i;
//		}
//
//		// Reset storing vectors
//		bandConfidence = std::vector<double>(permutationNumber, 1);
//		bandDistance = std::vector<double>(permutationNumber, 0);
//	}

	return Permutation(minDistance, minConfidence, minIndex);
}

