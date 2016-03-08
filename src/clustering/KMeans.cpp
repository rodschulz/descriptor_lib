/**
 * Author: rodrigo
 * 2015
 */
#include "KMeans.hpp"

#include <limits>
#include <boost/random.hpp>

boost::random::mt19937 randomGen;

template<typename T>
void print(cv::Mat mat, int prec = 0)
{
	std::cout << "[";
	for (int i = 0; i < mat.size().height; i++)
	{
		std::cout << " ";
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

KMeans::KMeans()
{
}

KMeans::~KMeans()
{
}

std::vector<int> KMeans::getRandomSet(const unsigned int _size, const int _min, const int _max)
{
	randomGen.seed(std::rand());
	boost::random::uniform_int_distribution<> dist(_min, _max);

	std::vector<int> numbers;
	numbers.reserve(_size);

	std::map<int, bool> used;
	while (numbers.size() < _size)
	{
		int number = dist(randomGen);
		if (used.find(number) == used.end())
			numbers.push_back(number);
	}

	return numbers;
}

void KMeans::searchClusters(const cv::Mat &_items, const int _clusterNumber, const Metric &_metric, const int _attempts, const int _maxIterations, const double _threshold, cv::Mat &_labels, cv::Mat &_centers, std::vector<double> &_errorCurve)
{
	_centers = cv::Mat::zeros(_clusterNumber, _items.cols, CV_32FC1);
	_labels = cv::Mat::zeros(_items.rows, 1, CV_32SC1);
	std::vector<int> itemsPerCenter;
	_errorCurve.clear();

	double minSSE = std::numeric_limits<double>::max();
	for (int i = 0; i < _attempts; i++)
	{
		std::cout << "\t** attempt " << i << std::endl;

		cv::Mat centers = cv::Mat::zeros(_clusterNumber, _items.cols, CV_32FC1);
		cv::Mat labels = cv::Mat::zeros(_items.rows, 1, CV_32SC1);
		std::vector<double> sseCurve;

		// Select some of the elements as the staring points
		selectStartCenters(_items, centers);

		// Iterate until the desired max iterations
		std::vector<int> itemCount;
		for (int j = 0; j < _maxIterations; j++)
		{
			if (j % 50 == 0)
				std::cout << "\t\tit:" << j << std::endl;

			// Set labels
			for (int k = 0; k < _items.rows; k++)
				labels.at<int>(k) = findClosestCenter(_items.row(k), centers, _metric);

			// Store SSE evolution
			sseCurve.push_back(getSSE(_items, labels, centers, _metric));

			// Calculate updated centers and check has to stop
			cv::Mat newCenters = _metric.calculateCenters(_clusterNumber, _items, labels, itemCount);
			bool thresholdReached = evaluateStopCondition(centers, newCenters, _threshold, _metric);

			// Update centers
			newCenters.copyTo(centers);
			if (thresholdReached)
			{
				std::cout << "\tthreshold reached --> [attempt: " << i << " - iteration: " << j << "]" << std::endl;
				break;
			}
		}

		double sse = getSSE(_items, labels, centers, _metric);
		std::cout << "\tSSE: " << std::fixed << sse << std::endl;

		if (sse < minSSE)
		{
			labels.copyTo(_labels);
			centers.copyTo(_centers);
			minSSE = sse;
			itemsPerCenter = itemCount;
			_errorCurve = sseCurve;
		}
	}

	std::cout << "KMeans finished -- min SSE: " << minSSE << "\n";
	for (size_t i = 0; i < itemsPerCenter.size(); i++)
		std::cout << "\tcluster " << i << ": " << itemsPerCenter[i] << " points\n";
}

void KMeans::stochasticSearchClusters(const cv::Mat &_items, const int _clusterNumber, const int _sampleSize, const Metric &_metric, const int _attempts, const int _maxIterations, const double _threshold, cv::Mat &_labels, cv::Mat &_centers, std::vector<double> &_errorCurve)
{
	_centers = cv::Mat::zeros(_clusterNumber, _items.cols, CV_32FC1);
	_labels = cv::Mat::zeros(_items.rows, 1, CV_32SC1);

	double bestSSE = std::numeric_limits<double>::max();
	for (int i = 0; i < _attempts; i++)
	{
		std::cout << "\t** attempt " << i << std::endl;

		cv::Mat centers = cv::Mat::zeros(_clusterNumber, _items.cols, CV_32FC1);
		cv::Mat sample = cv::Mat::zeros(_sampleSize, _items.cols, CV_32SC1);
		cv::Mat sampleLabels = cv::Mat::zeros(_sampleSize, 1, CV_32SC1);

		// Select some of the elements as the staring points
		selectStartCenters(_items, centers);

		// Iterate until the desired max iterations
		for (int j = 0; j < _maxIterations; j++)
		{
			if (j % 50 == 0)
				std::cout << "\t\tit:" << j << std::endl;

			// Select sample points
			getSample(_items, sample);

			// Set labels
			for (int k = 0; k < sample.rows; k++)
				sampleLabels.at<int>(k) = findClosestCenter(sample.row(k), centers, _metric);

			// Calculate updated centers
			std::vector<int> itemCount;
			cv::Mat newCenters = _metric.calculateCenters(_clusterNumber, sample, sampleLabels, itemCount);

			// Evaluate the precision stop condition
			bool thresholdReached = evaluateStopCondition(centers, newCenters, _threshold, _metric);

			// Update centers data
			for (int k = 0; k < newCenters.rows; k++)
				if (itemCount[k] > 0)
					newCenters.row(k).copyTo(centers.row(k));

			if (thresholdReached)
			{
				std::cout << "\tthreshold reached --> [attempt: " << i << " - iteration: " << j << "]" << std::endl;
				break;
			}
		}

		// Get the labels for the whole set
		cv::Mat setLabels = cv::Mat::zeros(_items.rows, 1, CV_32SC1);
		for (int k = 0; k < _items.rows; k++)
			setLabels.at<int>(k) = findClosestCenter(_items.row(k), centers, _metric);

		double sse = getSSE(_items, setLabels, centers, _metric);
		std::cout << "\tSSE: " << std::fixed << sse << std::endl;

		// Update best clustering so far
		if (sse < bestSSE)
		{
			setLabels.copyTo(_labels);
			centers.copyTo(_centers);
			bestSSE = sse;
		}
	}
}

double KMeans::getSSE(const cv::Mat &_items, const cv::Mat &_labels, const cv::Mat &_centers, const Metric &_metric)
{
	double sse = 0;
	for (int i = 0; i < _items.rows; i++)
	{
		double norm = _metric.distance(_items.row(i), _centers.row(_labels.at<int>(i)));
		sse += (norm * norm);
	}

	return sse;
}

bool KMeans::evaluateStopCondition(const cv::Mat &_oldCenters, const cv::Mat &_newCenters, const double _threshold, const Metric &_metric)
{
	bool thresholdReached = true;
	for (int k = 0; k < _oldCenters.rows && thresholdReached; k++)
		thresholdReached = thresholdReached && (_metric.distance(_oldCenters.row(k), _newCenters.row(k)) < _threshold);

	return thresholdReached;
}

void KMeans::selectStartCenters(const cv::Mat &_items, cv::Mat &_centers)
{
	std::vector<int> randomSet = getRandomSet(_centers.rows, 0, _items.rows - 1);
	for (int j = 0; j < _centers.rows; j++)
		_items.row(randomSet[j]).copyTo(_centers.row(j));
}

int KMeans::findClosestCenter(const cv::Mat &_vector, cv::Mat &_centers, const Metric &_metric)
{
	int closestCenter = -1;

	// Find the closest centroid for the current descriptor
	double minDist = std::numeric_limits<double>::max();
	for (int i = 0; i < _centers.rows; i++)
	{
		double distance = _metric.distance(_vector, _centers.row(i));
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
	std::vector<int> randomSet = getRandomSet(_sample.rows, 0, _items.rows - 1);
	for (int j = 0; j < _sample.rows; j++)
		_items.row(randomSet[j]).copyTo(_sample.row(j));
}
