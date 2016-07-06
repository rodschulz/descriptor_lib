/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include "Metric.hpp"
#include <stdexcept>
#include <iostream>
#include <boost/lexical_cast.hpp>

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
	ClosestPermutationMetric(const int permutationSize_)
	{
		permutationSize = permutationSize_;
	}

	// Destructor
	~ClosestPermutationMetric()
	{
	}

	// Returns the distance between the given vectors, according to the current metric
	inline double distance(const cv::Mat &vector1_, const cv::Mat &vector2_) const
	{
		return getClosestPermutation(vector1_, vector2_).distance;
	}

	// Returns the central point amongst the given items, according to the given labels
	inline cv::Mat calculateCenters(const int clusterNumber_, const cv::Mat &items_, const cv::Mat &labels_, std::vector<int> &itemsPerCenter_) const
	{
		itemsPerCenter_ = std::vector<int>(clusterNumber_, 0);

		// Matrixes holding the first vector used to compare and the new centers
		cv::Mat firstVector = cv::Mat::zeros(clusterNumber_, items_.cols, CV_32FC1);
		cv::Mat newCenters = cv::Mat::zeros(clusterNumber_, items_.cols, CV_32FC1);

		// Iterate over labels calculating the
		std::vector<bool> begin(clusterNumber_, true);
		for (int i = 0; i < labels_.rows; i++)
		{
			int clusterIndex = labels_.at<int>(i);

			// Track if every centroid has got its first element
			if (begin[clusterIndex])
			{
				items_.row(i).copyTo(firstVector.row(clusterIndex));
				items_.row(i).copyTo(newCenters.row(clusterIndex));
				begin[clusterIndex] = false;
			}
			else
			{
				Permutation closestPermutation = getClosestPermutation(firstVector.row(clusterIndex), items_.row(i));

				// Add the row's value to new center
				newCenters.row(clusterIndex).colRange(0, newCenters.cols - (closestPermutation.index * permutationSize)) += items_.row(i).colRange(closestPermutation.index * permutationSize, newCenters.cols);
				newCenters.row(clusterIndex).colRange(newCenters.cols - (closestPermutation.index * permutationSize), newCenters.cols) += items_.row(i).colRange(0, closestPermutation.index * permutationSize);
			}

			itemsPerCenter_[clusterIndex] += 1;
		}

		/***** DEBUG *****/
		if (debugEnabled)
		{
			for (int i = 0; i < newCenters.rows; i++)
				if (itemsPerCenter_[i] == 0)
					std::cout << "Zero!!" << std::endl;
		}
		/***** DEBUG *****/

		for (int i = 0; i < newCenters.rows; i++)
			newCenters.row(i) /= itemsPerCenter_[i];

		validateCenters(newCenters);

		return newCenters;
	}

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

	// Returns the type of the current metric
	MetricType getType() const
	{
		return METRIC_CLOSEST_PERMUTATION;
	}

	// Returns the parameters used to construct the current instance
	std::vector<std::string> getConstructionParams() const
	{
		std::vector<std::string> params;
		params.push_back(boost::lexical_cast<std::string>(permutationSize));
		return params;
	}

	// Validates and fixes the given centers, according to the metric's definition
	inline void validateCenters(cv::Mat &centers_) const
	{
		bool valid;
		do
		{
			// Set flag
			valid = true;

			// Validate centers
			for (int i = 0; i < centers_.rows && valid; i++)
			{
				for (int j = i + 1; j < centers_.rows; j++)
				{
					// If there's an image center, then the closest permutarion should be quite close
					Permutation permutation = getClosestPermutation(centers_.row(i), centers_.row(j));

					/***** DEBUG *****/
					if (debugEnabled && permutation.distance < 1)
						std::cout << i << "-" << j << ": " << permutation.distance << std::endl;
					/***** DEBUG *****/

					if (permutation.distance < 1e-5)
					{
						std::cout << "WARNING: image centers (" << i << "-" << j << "), attempting fix" << std::endl;

						// Update centers to make them valid
						centers_.row(j) *= 3;
						centers_.row(j) /= 2;

						// Centers have to be revalidated since a change was done
						valid = false;
						break;
					}
				}
			}
		} while (!valid);
	}

protected:
	int permutationSize; // Size of the permutation used by this metric (number of elements moved for each permutation)
};
