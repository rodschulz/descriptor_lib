/**
 * Author: rodrigo
 * 2015
 */
#include "ClosestPermutation.h"
#include <iostream>

ClosestPermutation::ClosestPermutation(const int _permutationSize)
{
	permutationSize = _permutationSize;
}

ClosestPermutation::~ClosestPermutation()
{
}

double ClosestPermutation::distance(const cv::Mat &_vector1, const cv::Mat &_vector2)
{
	if (cachedVector.empty() || cachedVector.cols != _vector2.cols)
		cachedVector = _vector2.clone();
	else
		_vector2.copyTo(cachedVector);

	int permutationNumber = std::floor(_vector2.cols / permutationSize);

	double minDist = cv::norm(_vector1, cachedVector);
	for (int i = 1; i < permutationNumber; i++)
	{
		// Copy the piece of data from the current permutation to the end of the vector
		int begin = permutationSize * i * sizeof(float);
		int size = (_vector2.cols - (permutationSize * i)) * sizeof(float);
		memcpy(cachedVector.datastart, _vector2.datastart + begin, size);

		// Copy the piece of data from the begin till the permutation begin
		memcpy(cachedVector.datastart + size, _vector2.datastart, begin);

		double dist = cv::norm(_vector1, cachedVector);
		if (minDist > dist)
			minDist = dist;
	}

	return minDist;
}

cv::Mat ClosestPermutation::calculateCenters(const int _clusterNumber, const cv::Mat &_items, const cv::Mat &_labels)
{
	std::vector<int> itemCount;
	return calculateCenters(_clusterNumber, _items, _labels, itemCount);
}

cv::Mat ClosestPermutation::calculateCenters(const int _clusterNumber, const cv::Mat &_items, const cv::Mat &_labels, std::vector<int> &_itemsPerCenter)
{
	std::vector<bool> begin(_clusterNumber, true);
	_itemsPerCenter = std::vector<int>(_clusterNumber, 0);

	cv::Mat first = cv::Mat::zeros(_clusterNumber, _items.cols, CV_32FC1);
	cv::Mat newCenters = cv::Mat::zeros(_clusterNumber, _items.cols, CV_32FC1);

	for (int i = 0; i < _labels.rows; i++)
	{
		int index = _labels.at<int>(i);
		if (begin[index])
			_items.row(index).copyTo(first.row(index));
		begin[index];

		newCenters.row(index) += getClosestPermutation(first.row(index), _items.row(index));
		_itemsPerCenter[index] += 1;
	}

	for (int i = 0; i < newCenters.rows; i++)
		newCenters.row(i) /= _itemsPerCenter[i];

	return newCenters;
}

cv::Mat ClosestPermutation::getClosestPermutation(const cv::Mat &_vector1, const cv::Mat &_vector2)
{
	if (cachedVector.empty() || cachedVector.cols != _vector2.cols)
		cachedVector = _vector2.clone();
	else
		_vector2.copyTo(cachedVector);

	if (cachedPermutation.empty() || cachedPermutation.cols != _vector2.cols)
		cachedPermutation = _vector2.clone();
	else
		_vector2.copyTo(cachedPermutation);

	double minDist = cv::norm(_vector1, cachedVector);
	int permutationNumber = std::floor(_vector2.cols / permutationSize);
	for (int i = 1; i < permutationNumber; i++)
	{
		// Copy the piece of data from the current permutation to the end of the vector
		int begin = permutationSize * i * sizeof(float);
		int size = (_vector2.cols - (permutationSize * i)) * sizeof(float);
		memcpy(cachedVector.datastart, _vector2.datastart + begin, size);

		// Copy the piece of data from the begin till the permutation begin
		memcpy(cachedVector.datastart + size, _vector2.datastart, begin);

		double dist = cv::norm(_vector1, cachedVector);
		if (minDist > dist)
		{
			minDist = dist;
			cachedVector.copyTo(cachedPermutation);
		}
	}

	return cachedPermutation;
}
