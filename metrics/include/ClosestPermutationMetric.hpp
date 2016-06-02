/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include "Metric.hpp"
#include <stdexcept>

// Metric measuring the closest distance between two vectors, according to the given permutation size
class ClosestPermutationMetric: public Metric
{
public:
	// Structure defining the basic permutation
	struct Permutation
	{
		double distance;
		int index;

		Permutation()
		{
			distance = index = -1;
		}

		Permutation(const double _distance, int _index)
		{
			distance = _distance;
			index = _index;
		}
	};

	// Constructor
	ClosestPermutationMetric(const int _permutationSize);

	// Destructor
	~ClosestPermutationMetric();

	// Returns the distance between the given vectors, according to the current metric
	double distance(const cv::Mat &_vector1, const cv::Mat &_vector2) const;

	// Returns the central point amongst the given items, according to the given labels
	cv::Mat calculateCenters(const int _clusterNumber, const cv::Mat &_items, const cv::Mat &_labels, std::vector<int> &_itemsPerCenter) const;

	// Returns the closest permutation according to this metric's params
	inline Permutation getClosestPermutation(const cv::Mat &_vector1, const cv::Mat &_vector2) const
	{
		// Check if dimensions match
		if (_vector1.cols != _vector2.cols || _vector1.rows != _vector2.rows)
			throw std::runtime_error("Invalid matrix dimensions in distance");

		// Check if dimensions match
		if (_vector1.cols % permutationSize != 0 && _vector2.cols % permutationSize != 0)
			throw std::runtime_error("Permutation size incompatible with vector lenghts");

		// Check dimensions are right (row vectors expected)
		if (_vector1.cols == 1 || _vector2.cols == 1)
			throw std::runtime_error("Column vectors are not supported, only row vectors");

		// Results
		double minDistance = std::numeric_limits<double>::max();
		int minIndex = -1;

		// Iterate over every possible permutation
		double distance = 0;
		int permutationNumber = std::floor(_vector1.cols / permutationSize);
		for (int i = 0; i < permutationNumber; i++)
		{
			int baseIndex = i * permutationSize;
			for (int j = 0; j < _vector1.cols; j++)
			{
				float x1 = _vector1.at<float>(j);
				float x2 = _vector2.at<float>((baseIndex + j) % _vector1.cols);
				distance += (x1 - x2) * (x1 - x2);
			}
			distance = sqrt(distance);

			// Check if the permutation is closer
			if (distance < minDistance)
			{
				minDistance = distance;
				minIndex = i;
			}

			distance = 0;
		}

		return Permutation(minDistance, minIndex);
	}

private:
	int permutationSize; // Size of the permutation used by this metric (number of elements moved for each permutation)
};
