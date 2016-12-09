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
	// Performs the search of clusters using kmeans algorithm
	static void searchClusters(ClusteringResults &results_, const cv::Mat &items_, const MetricPtr &metric_, const int ncluster_, const int attempts_, const int maxIterations_, const double stopThreshold_);

	// Performs the search of clusters using a stochastic version of kmeans algorithm
	static void stochasticSearchClusters(ClusteringResults &results_, const cv::Mat &items_, const MetricPtr &metric_, const int ncluster_, const int attempts_, const int maxIterations_, const double stopThreshold_, const int sampleSize_);

private:
	// Constuctor
	KMeans();
	// Destructor
	~KMeans();

	// Runs the kmeans algorithm to find clusters in the given data
	static inline void run(ClusteringResults &_results, const cv::Mat &items_, const MetricPtr &metric_, const int _ncluster, const int _attempts, const int _maxIterations, const double _threshold, const int _sampleSize = -1);

	// Updates the possition of the centers according to the given data labels
	static inline bool updateCenters(cv::Mat &centers_, const cv::Mat &sample_, const cv::Mat labels_, const double stopThreshold_, const MetricPtr &metric_);

	// Finds the closest center to each item
	static inline int findClosestCenter(const cv::Mat &vector_, const cv::Mat &centers_, const MetricPtr &metric_);
};
