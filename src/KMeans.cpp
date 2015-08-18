/**
 * Author: rodrigo
 * 2015
 */
#include "KMeans.h"
#include <limits>

KMeans::KMeans()
{
}

KMeans::~KMeans()
{
}

void KMeans::findClusters(const cv::Mat &_descriptors, const int _clusterNumber, const Metric &_metric, const int _attempts, const int _maxIterations, const double _threshold, cv::Mat &_labels, cv::Mat &_centers)
{
	_centers = cv::Mat::zeros(_clusterNumber, _descriptors.cols, CV_32FC1);
	_labels = cv::Mat::zeros(_descriptors.rows, 1, CV_32FC1);

	double currentSSE = std::numeric_limits<double>::max();

	for (int i = 0; i < _attempts; i++)
	{
		cv::Mat centers = cv::Mat::zeros(_clusterNumber, _descriptors.cols, CV_32FC1);
		cv::Mat labels = cv::Mat::zeros(_descriptors.rows, 1, CV_32FC1);

		// Iterate until the desired max iterations
		for (int j = 0; j < _maxIterations; j++)
		{
			// Evaluate the precision stop condition
			if (true)
				break;
		}

		double sse = calculateSSE(_descriptors, labels, centers, _metric);
		if (sse < currentSSE)
		{
			labels.copyTo(_labels);
			centers.copyTo(_centers);
			currentSSE = sse;
		}
	}
}

double KMeans::calculateSSE(const cv::Mat &_descriptors, const cv::Mat &_labels, const cv::Mat &_centers, const Metric &_metric)
{
	double sse = 0;
	for (int i = 0; i < _labels.rows; i++)
	{
	}

	return sse;
}
