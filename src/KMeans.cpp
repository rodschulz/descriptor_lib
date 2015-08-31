/**
 * Author: rodrigo
 * 2015
 */
#include "KMeans.h"
#include <limits>
#include "Helper.h"

KMeans::KMeans()
{
}

KMeans::~KMeans()
{
}

void KMeans::findClusters(const cv::Mat &_descriptors, const int _clusterNumber, Metric &_metric, const int _attempts, const int _maxIterations, const double _threshold, cv::Mat &_labels, cv::Mat &_centers)
{
	_centers = cv::Mat::zeros(_clusterNumber, _descriptors.cols, CV_32FC1);
	_labels = cv::Mat::zeros(_descriptors.rows, 1, CV_32FC1);

	double currentSSE = std::numeric_limits<double>::max();

	for (int i = 0; i < _attempts; i++)
	{
		cv::Mat centers = cv::Mat::zeros(_clusterNumber, _descriptors.cols, CV_32FC1);
		cv::Mat labels = cv::Mat::zeros(_descriptors.rows, 1, CV_32FC1);

		// Select some of the elements as the staring points
		selectStartCenters(_descriptors, centers);

		// Iterate until the desired max iterations
		for (int j = 0; j < _maxIterations; j++)
		{
			// Set labels
			for (int k = 0; k < _descriptors.rows; k++)
				labels.at<int>(k) = findClosestCenter(_descriptors.row(k), centers, _metric);

			// Update centers
			cv::Mat newCenters = _metric.calculateCenters(_clusterNumber, _descriptors, labels);

			// Evaluate the precision stop condition
			bool stop = true;
			for (int k = 0; k < centers.rows && stop; k++)
				stop = stop && (_metric.distance(centers.row(k), newCenters.row(k)) < _threshold);

			if (stop)
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

void KMeans::selectStartCenters(const cv::Mat &_descriptors, cv::Mat &_centers)
{
	std::map<int, bool> selectedElements;
	for (int j = 0; j < _centers.rows; j++)
	{
		// Get a random index not used so far
		int index = Helper::getRandomNumber(0, _descriptors.rows);
		while (selectedElements.find(index) != selectedElements.end())
			index = Helper::getRandomNumber(0, _descriptors.rows);

		_descriptors.row(index).copyTo(_centers.row(j));
		selectedElements[index] = true;
	}
}

double KMeans::calculateSSE(const cv::Mat &_descriptors, const cv::Mat &_labels, const cv::Mat &_centers, Metric &_metric)
{
	double sse = 0;
	for (int i = 0; i < _descriptors.rows; i++)
	{
		double norm = _metric.distance(_descriptors.row(i), _centers.row(_labels.at<int>(i)));
		sse += (norm * norm);
	}

	return sse;
}

int KMeans::findClosestCenter(const cv::Mat &_descriptor, cv::Mat &_centers, Metric &_metric)
{
	int closestCenter = -1;

	// Find the closest centroid for the current descriptor
	double minDist = std::numeric_limits<double>::max();
	for (int i = 0; i < _centers.rows; i++)
	{
		double distance = _metric.distance(_descriptor, _centers.row(i));
		if (distance < minDist)
		{
			minDist = distance;
			closestCenter = i;
		}
	}

	return closestCenter;
}
