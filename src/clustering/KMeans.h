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
	static void searchClusters(const cv::Mat &_descriptors, const int _clusterNumber, Metric &_metric, const int _attempts, const int _maxIterations, const double _threshold, cv::Mat &_labels, cv::Mat &_centers);
	static void stochasticSearchClusters(const cv::Mat &_descriptors, const int _clusterNumber, const int _sampleSize, Metric &_metric, const int _attempts, const int _maxIterations, const double _threshold, cv::Mat &_labels, cv::Mat &_centers);

	static bool loadClusters(const std::string _filename, cv::Mat &_centers, cv::Mat &_labels);
	static bool saveClusters(const cv::Mat &_centers, const cv::Mat &_labels, const std::string _filename);

private:
	KMeans();
	~KMeans();

	static bool evaluateStopCondition(const cv::Mat &_oldCenters, const cv::Mat &_newCenters, const double _threshold, Metric &_metric);
	static void selectStartCenters(const cv::Mat &_descriptors, cv::Mat &_centers);
	static int findClosestCenter(const cv::Mat &_descriptor, cv::Mat &_centers, Metric &_metric);
	static double calculateSSE(const cv::Mat &_descriptors, const cv::Mat &_labels, const cv::Mat &_centers, Metric &_metric);


	static void getSample(const cv::Mat &_descriptors, cv::Mat &_sample);
};
