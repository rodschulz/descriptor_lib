/**
 * Author: rodrigo
 * 2016
 */
#pragma once

#include <opencv2/core/core.hpp>
#include "Metric.hpp"
#include "Clustering.hpp"

class KMedoids
{
public:
	// Performs the search of clusters using kmeans algorithm
	static void searchClusters(ClusteringResults &results_, const cv::Mat &items_, const MetricPtr &metric_, const int ncluster_, const int attempts_, const int maxIterations_, const double stopThreshold_);

private:
	// Constuctor
	KMedoids();
	// Destructor
	~KMedoids();

	// Finds the closest center to each item
	static inline int findClosestCenter(const cv::Mat &items_, const int target_, const std::vector<int> &medoids_, const MetricPtr &metric_, cv::Mat &distances_);

	// Extracts the given medoids
	static inline void extractMedoids(const cv::Mat &items_, const std::vector<int> &indices_, cv::Mat &medoids_);
};
