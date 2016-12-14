/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <opencv2/core/core.hpp>
#include "Metric.hpp"
#include "Clustering.hpp"


class KMeans
{
public:
	/**************************************************/
	static void searchClusters(ClusteringResults &results_,
							   const cv::Mat &items_,
							   const MetricPtr &metric_,
							   const int ncluster_,
							   const int attempts_,
							   const int maxIterations_,
							   const double stopThreshold_);

	/**************************************************/
	static void stochasticSearchClusters(ClusteringResults &results_,
										 const cv::Mat &items_,
										 const MetricPtr &metric_,
										 const int ncluster_,
										 const int attempts_,
										 const int maxIterations_,
										 const double stopThreshold_,
										 const int sampleSize_);

private:
	KMeans();
	~KMeans();

	/**************************************************/
	static inline void run(ClusteringResults &results_,
						   const cv::Mat &items_,
						   const MetricPtr &metric_,
						   const int ncluster_,
						   const int attempts_,
						   const int maxIterations_,
						   const double threshold_,
						   const int sampleSize_ = -1);

	/**************************************************/
	static inline bool updateCenters(cv::Mat &centers_,
									 const cv::Mat &sample_,
									 const cv::Mat labels_,
									 const double stopThreshold_,
									 const MetricPtr &metric_);

	/**************************************************/
	static inline int findClosestCenter(const cv::Mat &vector_,
										const cv::Mat &centers_,
										const MetricPtr &metric_);
};
