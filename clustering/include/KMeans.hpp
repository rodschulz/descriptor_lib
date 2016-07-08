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
	static void searchClusters(ClusteringResults &_results, const cv::Mat &_items, const MetricPtr &_metric, const int _ncluster, const int _attempts, const int _maxIterations, const double _stopThreshold);

	// Performs the search of clusters using a stochastic version of kmeans algorithm
	static void stochasticSearchClusters(ClusteringResults &_results, const cv::Mat &_items, const MetricPtr &_metric, const int _ncluster, const int _attempts, const int _maxIterations, const double _stopThreshold, const int _sampleSize);

	// Calculates the Sum of Squared Errors for the given centers and labels, using the given metric
	static inline double getSSE(const cv::Mat &_items, const cv::Mat &_labels, const cv::Mat &_centers, const MetricPtr &_metric)
	{
		double sse = 0;
		for (int i = 0; i < _items.rows; i++)
		{
			double norm = _metric->distance(_items.row(i), _centers.row(_labels.at<int>(i)));
			sse += (norm * norm);
		}

		return sse;
	}

private:
	// Constuctor
	KMeans();
	// Destructor
	~KMeans();

	// Runs the kmeans algorithm to find clusters in the given data
	static inline void run(ClusteringResults &_results, const cv::Mat &_items, const MetricPtr &_metric, const int _ncluster, const int _attempts, const int _maxIterations, const double _threshold, const int _sampleSize = -1);

	// Updates the possition of the centers according to the given data labels
	static inline bool updateCenters(cv::Mat &centers_, const cv::Mat &sample_, const cv::Mat labels_, const double stopThreshold_, const MetricPtr &metric_);

	// Evaluates if the stop condition has been met for the current data
	static inline bool evaluateStopCondition(const cv::Mat &oldCenters_, const cv::Mat &newCenters_, const double stopThreshold_, const MetricPtr &metric_);

	// Finds the closest center to each item
	static inline int findClosestCenter(const cv::Mat &_items, const cv::Mat &_centers, const MetricPtr &_metric);

	// Retrieves a sample of data from the given items matrix
	static inline void getSample(const cv::Mat &_items, cv::Mat &_sample);

	// Counts the number of items per centers according to the given labeling
	static inline std::vector<int> itemsPerCluster(const int clusterNumber_, const cv::Mat &labels_)
	{
		 std::vector<int> count(clusterNumber_, 0);
		 for (int i = 0; i < labels_.rows; i++)
			 count[labels_.at<int>(i)]++;
		 return count;
	}
};
