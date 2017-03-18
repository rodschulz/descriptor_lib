/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include "Metric.hpp"
#include <stdexcept>
#include <iostream>
#include <boost/lexical_cast.hpp>
#include "Utils.hpp"


class ClosestPermutationMetric: public Metric
{
public:
	/**************************************************/
	struct Permutation
	{
		double distance;
		int index;

		Permutation()
		{
			distance = index = -1;
		}

		Permutation(const double _distance, int index_)
		{
			distance = _distance;
			index = index_;
		}
	};

	/**************************************************/
	ClosestPermutationMetric(const int permutationSize_)
	{
		permutationSize = permutationSize_;
	}
	~ClosestPermutationMetric() {}

	/**************************************************/
	inline double distance(const cv::Mat &vector1_,
						   const cv::Mat &vector2_) const
	{
		return getClosestPermutation(vector1_, vector2_).distance;
	}

	/**************************************************/
	inline cv::Mat calculateMeans(const int clusterNumber_,
								  const cv::Mat &items_,
								  const cv::Mat &labels_,
								  const cv::Mat &currentMeans_ = cv::Mat()) const
	{
		std::vector<int> itemCount = std::vector<int>(clusterNumber_, 0);

		cv::Mat newMeans = cv::Mat::zeros(clusterNumber_, items_.cols, CV_32FC1);
		for (int i = 0; i < labels_.rows; i++)
		{
			int cluster = labels_.at<int>(i);
			Permutation closest = getClosestPermutation(currentMeans_.row(cluster), items_.row(i));

			// Add the row's value to new center
			newMeans.row(cluster).colRange(0, newMeans.cols - (closest.index * permutationSize)) += items_.row(i).colRange(closest.index * permutationSize, newMeans.cols);
			newMeans.row(cluster).colRange(newMeans.cols - (closest.index * permutationSize), newMeans.cols) += items_.row(i).colRange(0, closest.index * permutationSize);

			itemCount[cluster] += 1;
		}


		/***** DEBUG *****/
		if (debugEnabled)
		{
			for (int i = 0; i < newMeans.rows; i++)
				if (itemCount[i] == 0)
					std::cout << "Zero!!" << std::endl;
		}
		/***** DEBUG *****/


		for (int i = 0; i < newMeans.rows; i++)
		{
			if (itemCount[i] != 0)
				newMeans.row(i) /= itemCount[i];
			else
			{
				int newCentroid = Utils::getRandomNumber(0, items_.rows);
				items_.row(newCentroid).copyTo(newMeans.row(i));
			}
		}

		validateMeans(newMeans);

		return newMeans;
	}


	/**************************************************/
	inline Permutation getClosestPermutation(const cv::Mat &vector1_,
			const cv::Mat &vector2_) const
	{
		// Check if dimensions match
		if (vector1_.cols != vector2_.cols || vector1_.rows != vector2_.rows)
			throw std::runtime_error("Invalid matrix dimensions in distance");

		// Check if dimensions match
		if (vector1_.cols % permutationSize != 0 && vector2_.cols % permutationSize != 0)
			throw std::runtime_error("Permutation size incompatible with vector lenghts");

		// Check dimensions are right (row vectors expected)
		if (vector1_.cols == 1 || vector2_.cols == 1)
			throw std::runtime_error("Column vectors are not supported, only row vectors");

		// Results
		double minDistance = std::numeric_limits<double>::max();
		int minIndex = -1;

		// Iterate over every possible permutation
		double distance = 0;
		int permutationNumber = std::floor(vector1_.cols / permutationSize);
		for (int i = 0; i < permutationNumber; i++)
		{
			int baseIndex = i * permutationSize;
			for (int j = 0; j < vector1_.cols; j++)
			{
				float x1 = vector1_.at<float>(j);
				float x2 = vector2_.at<float>((baseIndex + j) % vector1_.cols);
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


	/**************************************************/
	MetricType getType() const
	{
		return METRIC_CLOSEST_PERMUTATION;
	}


	/**************************************************/
	std::vector<std::string> getConstructionParams() const
	{
		std::vector<std::string> params;
		params.push_back(boost::lexical_cast<std::string>(permutationSize));
		return params;
	}


	/**************************************************/
	inline void validateMeans(cv::Mat &means_) const
	{
		bool valid;
		do
		{
			// Set flag
			valid = true;

			// Validate centers
			for (int i = 0; i < means_.rows && valid; i++)
			{
				for (int j = i + 1; j < means_.rows; j++)
				{
					// If there's an image center, then the closest permutation should be quite close
					Permutation permutation = getClosestPermutation(means_.row(i), means_.row(j));


					/***** DEBUG *****/
					if (debugEnabled && permutation.distance < 1)
						std::cout << i << "-" << j << ": " << permutation.distance << std::endl;
					/***** DEBUG *****/


					if (permutation.distance < 1e-5)
					{
						std::cout << "WARNING: image centers (" << i << "-" << j << "), attempting fix" << std::endl;

						// Update centers to make them valid
						means_.row(j) *= 3;
						means_.row(j) /= 2;

						// Centers have to be revalidated since a change was done
						valid = false;
						break;
					}
				}
			}
		} while (!valid);
	}


	/**************************************************/
	int getPermutationSize() const
	{
		return permutationSize;
	}


protected:
	int permutationSize;
};
