/**
 * Author: rodrigo
 * 2016     
 */
#pragma once

#include <opencv2/core/core.hpp>
#include "Metric.hpp"

class ClusteringUtils
{
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
	static inline void labelData(const cv::Mat &items_, const CvSVM &classifier_, cv::Mat &labels_)
	{
		svm->predict(items_, labels_);
	}
};
