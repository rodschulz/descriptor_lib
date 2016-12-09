/**
 * Author: rodrigo
 * 2016
 */
#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/ml/ml.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <stdio.h>
#include "Metric.hpp"
#include "Utils.hpp"

// shared pointers
typedef boost::shared_ptr<cv::SVM> SVMPtr;
typedef boost::shared_ptr<cv::Boost> BoostingPtr;
typedef boost::shared_ptr<cv::NeuralNet_MLP> NeuralNetworkPtr;

class ClusteringUtils
{
public:
	// Prints a matrix in a formated way
	template<typename T>
	static void print(cv::Mat mat, const int precision_ = 3)
	{
		std::string format;
		if (typeid(T) == typeid(float) || typeid(T) == typeid(double))
			format = "%." + boost::lexical_cast<std::string>(precision_) + "f";
		else
			format = "%d";

		std::cout << "[";
		for (int i = 0; i < mat.rows; i++)
		{
			std::cout << (i == 0 && mat.at<T>(0, 0) < 0 ? "" : " ");
			for (int j = 0; j < mat.cols; j++)
			{
				T num = mat.at<T>(i, j);
				if (num < 0 || (i == 0 && j == 0))
					printf(format.c_str(), num);
				else
					printf((" " + format).c_str(), num);

				if (j != mat.cols - 1)
					std::cout << "\t";
				else
					std::cout << (i == mat.rows - 1 ? "]" : "") << std::endl;
			}
		}
	}

	// Generates a permutation of the given matrix
	static inline void generatePermutation(const cv::Mat &matrix_,
										   const int permutationSize_,
										   const int permutationNumber_,
										   cv::Mat &permutation_)
	{
		int begin = permutationNumber_ * permutationSize_;
		int end = matrix_.cols;
		matrix_.colRange(begin, end).copyTo(permutation_.colRange(0, end - begin));

		begin = 0;
		end = permutationNumber_ * permutationSize_;
		if (end - begin > 0)
			matrix_.colRange(begin, end).copyTo(permutation_.colRange(permutation_.cols - (end - begin), permutation_.cols));
	}

	// Labels the given data using the given centers
	static inline void labelData(const cv::Mat &items_,
								 const cv::Mat &centers_,
								 const MetricPtr &metric_,
								 cv::Mat &labels_)
	{
		labels_ = cv::Mat::zeros(items_.rows, 1, CV_32SC1);
		std::vector<double> distance(items_.rows, std::numeric_limits<double>::max());
		for (int i = 0; i < items_.rows; i++)
		{
			for (int j = 0; j < centers_.rows; j++)
			{
				double dist = metric_->distance(centers_.row(j), items_.row(i));
				if (dist < distance[i])
				{
					distance[i] = dist;
					labels_.at<int>(i) = j;
				}
			}
		}
	}

	// Labels the given data using the given SVN as a classifier indicating the label
	static inline void labelData(const cv::Mat &items_,
								 const SVMPtr &classifier_,
								 cv::Mat &labels_)
	{
		classifier_->predict(items_, labels_);
	}

	// Prepares the classifier for the labeling process
	static SVMPtr prepareClassifier(const cv::Mat &centers_,
									const std::map<std::string,
									std::string> &centersParams_)
	{
		std::vector<std::string> params;
		std::string metricParams = centersParams_.at("metric");
		boost::trim_if(metricParams, boost::is_any_of("[]"));
		boost::split(params, metricParams, boost::is_any_of("[,]"), boost::token_compress_on);

		int permutationSize = centers_.cols;
		if (boost::iequals(params[0], metricType[METRIC_CLOSEST_PERMUTATION]) || boost::iequals(params[0], metricType[METRIC_CLOSEST_PERMUTATION_WITH_CONFIDENCE]))
			permutationSize = boost::lexical_cast<int>(params[1]);

		int permutationNumber = centers_.cols / permutationSize;

		// Prepare the training data
		cv::Mat trainingData = cv::Mat::zeros(centers_.rows * permutationNumber, centers_.cols, CV_32FC1);
		cv::Mat labels = cv::Mat::zeros(centers_.rows * permutationNumber, 1, CV_32FC1);

		for (int i = 0; i < centers_.rows; i++)
		{
			cv::Mat currentCentroid = centers_.row(i);
			for (int j = 0; j < permutationNumber; j++)
			{
				int index = i * permutationNumber + j;

				cv::Mat permutation = trainingData.row(index);
				generatePermutation(currentCentroid, permutationSize, j, permutation);
				labels.at<float>(index, 0) = (float) (i + 1);
			}
		}

		// Set SVM parameters
		CvSVMParams svmParams;
		svmParams.svm_type = cv::SVM::C_SVC;
		svmParams.kernel_type = cv::SVM::RBF; //cv::SVM::LINEAR;
		svmParams.term_crit = cvTermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 1000, 1e-8);

		// Train the SVM
		SVMPtr svm = SVMPtr(new cv::SVM());
		svm->train_auto(trainingData, labels, cv::Mat(), cv::Mat(), svmParams);

		return svm;
	}

	// Calculates the Sum of Squared Errors for the given centers and labels, using the given metric
	static inline double getSSE(const cv::Mat &items_,
								const cv::Mat &labels_,
								const cv::Mat &centers_,
								const MetricPtr &metric_)
	{
		double sse = 0;
		for (int i = 0; i < items_.rows; i++)
		{
			double norm = metric_->distance(items_.row(i), centers_.row(labels_.at<int>(i)));
			sse += (norm * norm);
		}

		return sse;
	}

	// Retrieves a sample of data from the given items matrix
	static inline void getSampleItems(const cv::Mat &items_,
									  cv::Mat &_sample)
	{
		std::vector<int> randomSet = Utils::getRandomIntArray(_sample.rows, 0, items_.rows - 1, false);
		for (int j = 0; j < _sample.rows; j++)
			items_.row(randomSet[j]).copyTo(_sample.row(j));
	}

	// Counts the number of items per centers according to the given labeling
	static inline std::vector<int> itemsPerCluster(const int clusterNumber_,
			const cv::Mat &labels_)
	{
		std::vector<int> count(clusterNumber_, 0);
		for (int i = 0; i < labels_.rows; i++)
			count[labels_.at<int>(i)]++;
		return count;
	}

	// Evaluates if the stop condition has been met for the current data
	static inline bool evaluateStopCondition(const cv::Mat &oldCenters_,
			const cv::Mat &newCenters_,
			const double stopThreshold_,
			const MetricPtr &metric_)
	{
		bool thresholdReached = true;
		for (int k = 0; k < oldCenters_.rows && thresholdReached; k++)
			thresholdReached = thresholdReached && (metric_->distance(oldCenters_.row(k), newCenters_.row(k)) < stopThreshold_);

		return thresholdReached;
	}
};
