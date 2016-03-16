/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <opencv2/core/core.hpp>

#include "Metric.hpp"

class KMeans
{
public:
	static void searchClusters(const cv::Mat &_descriptors, const int _clusterNumber, const Metric &_metric, const int _attempts, const int _maxIterations, const double _threshold, cv::Mat &_labels, cv::Mat &_centers, std::vector<double> &_errorCurve);
	static void stochasticSearchClusters(const cv::Mat &_descriptors, const int _clusterNumber, const int _sampleSize, const Metric &_metric, const int _attempts, const int _maxIterations, const double _threshold, cv::Mat &_labels, cv::Mat &_centers, std::vector<double> &_errorCurve);

//	static bool loadClusters(const std::string _filename, cv::Mat &_centers, cv::Mat &_labels);
//	static bool saveClusters(const cv::Mat &_centers, const cv::Mat &_labels, const std::string _filename);

	// Calculates the Sum of Squared Errors for the given centers and labels, using the given metric
	static double getSSE(const cv::Mat &_descriptors, const cv::Mat &_labels, const cv::Mat &_centers, const Metric &_metric);

private:
	KMeans();
	~KMeans();

	static std::vector<int> getRandomSet(const unsigned int _size, const int _min, const int _max);
	static bool evaluateStopCondition(const cv::Mat &_oldCenters, const cv::Mat &_newCenters, const double _threshold, const Metric &_metric);
	static void selectStartCenters(const cv::Mat &_descriptors, cv::Mat &_centers);
	static int findClosestCenter(const cv::Mat &_descriptor, cv::Mat &_centers, const Metric &_metric);

	static void getSample(const cv::Mat &_descriptors, cv::Mat &_sample);
};