/**
 * Author: rodrigo
 * 2015
 */
#include "KMeans.hpp"
#include <limits>
#include <stdio.h>
#include "Utils.hpp"

template<typename T>
void DEBUG_print(cv::Mat mat)
{
	std::cout << "[";
	for (int i = 0; i < mat.size().height; i++)
	{
		std::cout << (i == 0 && mat.at<T>(0, 0) < 0 ? "" : " ");
		for (int j = 0; j < mat.size().width; j++)
		{
			float num = mat.at<T>(i, j);
			if (num < 0 || (i == 0 && j == 0))
				printf("%.3f", num);
			else
				printf(" %.3f", num);

			if (j != mat.size().width - 1)
				std::cout << "\t";
			else
				std::cout << std::endl;
		}
	}
	std::cout << "]" << std::endl;
}

void KMeans::searchClusters(ClusteringResults &_results, const cv::Mat &_items, const MetricPtr &_metric, const int _ncluster, const int _attempts, const int _maxIterations, const double _stopThreshold)
{
	KMeans::run(_results, _items, _metric, _ncluster, _attempts, _maxIterations, _stopThreshold);
}

void KMeans::stochasticSearchClusters(ClusteringResults &_results, const cv::Mat &_items, const MetricPtr &_metric, const int _ncluster, const int _attempts, const int _maxIterations, const double _stopThreshold, const int _sampleSize)
{
	KMeans::run(_results, _items, _metric, _ncluster, _attempts, _maxIterations, _stopThreshold, _sampleSize);
}

void KMeans::run(ClusteringResults &_results, const cv::Mat &_items, const MetricPtr &_metric, const int _ncluster, const int _attempts, const int _maxIterations, const double _stopThreshold, const int _sampleSize)
{
	_results.prepare(_ncluster, _items.rows, _items.cols);
	std::vector<int> itemsPerCenter;

	double minSSE = std::numeric_limits<double>::max();
	for (int i = 0; i < _attempts; i++)
	{
		std::cout << "\t** attempt " << i << std::endl;

		cv::Mat centers = cv::Mat::zeros(_ncluster, _items.cols, CV_32FC1);
		cv::Mat sample = _sampleSize == -1 ? _items : cv::Mat::zeros(_sampleSize, _items.cols, CV_32SC1);
		cv::Mat labels = cv::Mat::zeros(sample.rows, 1, CV_32SC1);
		std::vector<double> sseCurve;

		// Select some of the elements as the initial centers
		getSample(_items, centers);

		// Iterate until the desired max iterations
		std::vector<int> itemCount;
		for (int j = 0; j < _maxIterations; j++)
		{
			if (j % 50 == 0)
				std::cout << "\t\tit:" << j << std::endl;

			// Select sample points (if needed)
			if (_sampleSize != -1)
				getSample(_items, sample);

			// Set labels
			for (int k = 0; k < sample.rows; k++)
				labels.at<int>(k) = findClosestCenter(sample.row(k), centers, _metric);

			// Store SSE evolution
			sseCurve.push_back(KMeans::getSSE(_items, labels, centers, _metric));

			// Updated centers and check if to stop
			if (updateCenters(itemCount, centers, sample, labels, _ncluster, _stopThreshold, _metric))
			{
				std::cout << "\tthreshold reached --> [attempt: " << i << " - iteration: " << j << "]" << std::endl;
				break;
			}
		}

		double sse = KMeans::getSSE(_items, labels, centers, _metric);
		std::cout << "\tSSE: " << std::fixed << sse << std::endl;

		if (sse < minSSE)
		{
			// Update the results
			labels.copyTo(_results.labels);
			centers.copyTo(_results.centers);
			_results.errorEvolution = sseCurve;

			// Update the control variables
			itemsPerCenter = itemCount;
			minSSE = sse;
		}
	}

	// Print a short report of the results
	std::cout << "KMeans finished -- SSE: " << minSSE << "\n";
	for (size_t i = 0; i < itemsPerCenter.size(); i++)
		std::cout << "\tcluster " << i << ": " << itemsPerCenter[i] << " points\n";
}

bool KMeans::updateCenters(std::vector<int> &_itemCount, cv::Mat &_centers, const cv::Mat &_sample, const cv::Mat _labels, const int _ncluster, const double _stopThreshold, const MetricPtr &_metric)
{
	// Calculate updated positions for the centers
	cv::Mat newCenters = _metric->calculateCenters(_ncluster, _sample, _labels, _itemCount);

	// Check if the "displacement" threshold was reached
	bool stop = evaluateStopCondition(_centers, newCenters, _stopThreshold, _metric);

	// Update centers
	newCenters.copyTo(_centers);

	// Return the stop
	return stop;
}

bool KMeans::evaluateStopCondition(const cv::Mat &_oldCenters, const cv::Mat &_newCenters, const double _threshold, const MetricPtr &_metric)
{
	bool thresholdReached = true;
	for (int k = 0; k < _oldCenters.rows && thresholdReached; k++)
		thresholdReached = thresholdReached && (_metric->distance(_oldCenters.row(k), _newCenters.row(k)) < _threshold);

	return thresholdReached;
}

int KMeans::findClosestCenter(const cv::Mat &_vector, const cv::Mat &_centers, const MetricPtr &_metric)
{
	int closestCenter = -1;

	// Find the closest centroid for the current descriptor
	double minDist = std::numeric_limits<double>::max();
	for (int i = 0; i < _centers.rows; i++)
	{
		double distance = _metric->distance(_vector, _centers.row(i));
		if (distance < minDist)
		{
			minDist = distance;
			closestCenter = i;
		}
	}

	return closestCenter;
}

void KMeans::getSample(const cv::Mat &_items, cv::Mat &_sample)
{
	std::vector<int> randomSet = Utils::getRandomArray(_sample.rows, 0, _items.rows - 1);
	for (int j = 0; j < _sample.rows; j++)
		_items.row(randomSet[j]).copyTo(_sample.row(j));
}
