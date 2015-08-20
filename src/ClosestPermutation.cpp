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

double ClosestPermutation::distance(const cv::Mat &_vector1, const cv::Mat &_vector2) const
{
	cv::Mat vector2 = _vector2.clone();
	int permutationNumber = std::floor(_vector2.cols / permutationSize);

	double minDist = cv::norm(_vector1, vector2);
	for (int i = 1; i < permutationNumber; i++)
	{
		// Copy the piece of data from the current permutation to the end of the vector
		int begin = permutationSize * i * sizeof(float);
		int size = (_vector2.cols - (permutationSize * i)) * sizeof(float);
		memcpy(vector2.datastart, _vector2.datastart + begin, size);

		// Copy the piece of data from the begin till the permutation begin
		memcpy(vector2.datastart + size, _vector2.datastart, begin);

		double dist = cv::norm(_vector1, vector2);
		if (minDist > dist)
			minDist = dist;
	}

	return minDist;
}

cv::Mat ClosestPermutation::calculateCenters(const int _clusterNumber, const cv::Mat &_descriptors, const cv::Mat &_labels) const
{
	cv::Mat first = cv::Mat::zeros(_clusterNumber, _descriptors.cols, CV_32FC1);
	cv::Mat newCenters = cv::Mat::zeros(_clusterNumber, _descriptors.cols, CV_32FC1);
	cv::Mat count = cv::Mat::zeros(_clusterNumber, 1, CV_32SC1);

	for (int i = 0; i < _labels.rows; i++)
	{
		int index = _labels.at<int>(i);
		if (first.row(index).at<float>(1) == 0)
			_descriptors.row(index).copyTo(first.row(index));

		newCenters.row(index) += getClosestPermutation(first.row(index), _descriptors.row(index));
		count.row(index) += 1;
	}

	for (int i = 0; i < newCenters.rows; i++)
	{
		int index = _labels.at<int>(i);
		newCenters.row(index) /= count.row(index).at<int>(0);
	}

	return newCenters;
}

cv::Mat ClosestPermutation::getClosestPermutation(const cv::Mat &_vector1, const cv::Mat &_vector2) const
{
	cv::Mat vector2 = _vector2.clone();
	int permutationNumber = std::floor(_vector2.cols / permutationSize);

	double minDist = cv::norm(_vector1, vector2);
	cv::Mat minPermutation = vector2.clone();

	for (int i = 1; i < permutationNumber; i++)
	{
		// Copy the piece of data from the current permutation to the end of the vector
		int begin = permutationSize * i * sizeof(float);
		int size = (_vector2.cols - (permutationSize * i)) * sizeof(float);
		memcpy(vector2.datastart, _vector2.datastart + begin, size);

		// Copy the piece of data from the begin till the permutation begin
		memcpy(vector2.datastart + size, _vector2.datastart, begin);

		double dist = cv::norm(_vector1, vector2);
		if (minDist > dist)
		{
			minDist = dist;
			vector2.copyTo(minPermutation);
		}
	}

	return minPermutation;
}
