/**
 * Author: rodrigo
 * 2016
 */
#pragma once

#include <opencv2/core/core.hpp>
#include "../metrics/Metric.hpp"
#include "Clustering.hpp"
#include <stdlib.h>

enum LinkageCriterion
{
	LINKAGE_SINGLE,
	LINKAGE_COMPLETE,
//	LINKAGE_GROUP_AVERAGE,
//	LINKAGE_CENTROID,
//	LINKAGE_WARD,
};

class Hierarchical
{
public:
	static void agglomerativeClustering(ClusteringResults &_results, const cv::Mat &_items, const MetricPtr &_metric);

private:
	// Constructor
	Hierarchical();
	// Destructor
	~Hierarchical();

	// Returns the index of the two closest clusters
	static inline std::pair<int, int> findClosetsClusters(const cv::Mat &_distanceMatrix);

	static inline void mergeClusters(const cv::Mat &_distanceMatrix, const int _cluster1, const int _cluster2);
};
