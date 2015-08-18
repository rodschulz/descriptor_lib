/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <opencv2/core/core.hpp>
#include "Metric.h"

class KMeans
{
public:
	static void findClusters(const cv::Mat &_descriptors, const int _clusterNumber, const Metric &_metric, const int _attempts, const int _maxIterations, const double _threshold, cv::Mat &_labels, cv::Mat &_centers);

private:
	KMeans();
	~KMeans();

	static double calculateSSE(const cv::Mat &_descriptors, const cv::Mat &_labels, const cv::Mat &_centers, const Metric &_metric);
};
