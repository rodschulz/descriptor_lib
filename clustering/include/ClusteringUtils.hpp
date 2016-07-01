/**
 * Author: rodrigo
 * 2016     
 */
#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/ml/ml.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include "Metric.hpp"

// SVM's shared pointer
typedef boost::shared_ptr<CvSVM> CvSVMPtr;

class ClusteringUtils
{
private:
	// Generates a permutation of the given matrix
	static inline void generatePermutation(const cv::Mat &matrix_, const int permutationSize_, const int permutationNumber_, cv::Mat &permutation_)
	{
		int begin = permutationNumber_ * permutationSize_;
		int end = matrix_.cols;
		matrix_.colRange(begin, end).copyTo(permutation_.colRange(0, end - begin));

		begin = 0;
		end = permutationNumber_ * permutationSize_;
		if (end - begin > 0)
			matrix_.colRange(begin, end).copyTo(permutation_.colRange(permutation_.cols - (end - begin), permutation_.cols));
	}

public:
	// Labels the given data using the given centers
	static inline void labelData(const cv::Mat &items_, const cv::Mat &centers_, const MetricPtr &metric_, cv::Mat &labels_)
	{
		// TODO implement a unit test for this method (probably over a syn cloud with some def centers and results)

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
	static inline void labelData(const cv::Mat &items_, const CvSVMPtr &classifier_, cv::Mat &labels_)
	{
		classifier_->predict(items_, labels_);
	}

	// Prepares the clasificator for the labeling process
	static CvSVMPtr prepareClasificator(const cv::Mat &centers_, const std::map<std::string, std::string> &centersParams_)
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
		svmParams.svm_type = CvSVM::C_SVC;
		svmParams.kernel_type = CvSVM::LINEAR;
		svmParams.term_crit = cvTermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 100, 1e-6);

		// Train the SVM
		CvSVMPtr svm = CvSVMPtr(new CvSVM());
		svm->train(trainingData, labels, cv::Mat(), cv::Mat(), svmParams);

		return svm;
	}
};
