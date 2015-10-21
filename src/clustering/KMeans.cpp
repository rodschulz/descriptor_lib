/**
 * Author: rodrigo
 * 2015
 */
#include "KMeans.h"
#include <limits>
#include "../utils/Helper.h"
#include "../io/Writer.h"

KMeans::KMeans()
{
}

KMeans::~KMeans()
{
}

void KMeans::searchClusters(const cv::Mat &_items, const int _clusterNumber, const Metric &_metric, const int _attempts, const int _maxIterations, const double _threshold, cv::Mat &_labels, cv::Mat &_centers)
{
	/**
	 * @TODO revisar que el new centers se este actualizando
	 *
	 * -hacer prueba de que el kmeans esta funcionado bien, para esto evaluar clusters de pocos puntos
	 * 	que esten bien separados y usar metrica euclideana.
	 * -hacer tests de funcionamiento de kmeans.
	 * -hacer matriz de distancias cruzadas (como una matriz de confusión) con todos
	 * 	los puntos => la matriz por bloques con las distancias que vimos en mineria
	 * -hacer una matriz de distancias, pero esta vez de la distancia con respecto al
	 * 	 promedio de los puntos dentro del cluster =>
	 *
	 */

	_centers = cv::Mat::zeros(_clusterNumber, _items.cols, CV_32FC1);
	_labels = cv::Mat::zeros(_items.rows, 1, CV_32FC1);
	std::vector<int> itemsPerCenter;

	std::vector<double> sseEvolution;

	double currentSSE = std::numeric_limits<double>::max();
	for (int i = 0; i < _attempts; i++)
	{
		std::cout << "\t** attempt " << i << std::endl;

		cv::Mat centers = cv::Mat::zeros(_clusterNumber, _items.cols, CV_32FC1);
		cv::Mat labels = cv::Mat::zeros(_items.rows, 1, CV_32FC1);

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
			sseEvolution.push_back(calculateSSE(_items, labels, centers, _metric));

			// Calculate updated centers
			cv::Mat newCenters = _metric.calculateCenters(_clusterNumber, _items, labels, itemCount);

			// Evaluate the stop condition
			bool thresholdReached = evaluateStopCondition(centers, newCenters, _threshold, _metric);

			// Update centers
			newCenters.copyTo(centers);

			if (thresholdReached)
			{
				std::cout << "\tthreshold reached --> [attempt: " << i << " - iteration: " << j << "]" << std::endl;
				break;
			}
		}

		double finalSSE = calculateSSE(_items, labels, centers, _metric);
		std::cout << "\tSSE: " << std::fixed << finalSSE << std::endl;

		if (finalSSE < currentSSE)
		{
			labels.copyTo(_labels);
			centers.copyTo(_centers);
			currentSSE = finalSSE;
			itemsPerCenter = itemCount;
		}
	}

	std::cout << "KMeans finished\n";
	for (size_t i = 0; i < itemsPerCenter.size(); i++)
		std::cout << "\tcluster " << i << ": " << itemsPerCenter[i] << " points\n";

	std::cout << "Generating SSE plot" << std::endl;
	Writer::writePlotSSE("sse", "SSE Evolution", sseEvolution);
}

void KMeans::stochasticSearchClusters(const cv::Mat &_items, const int _clusterNumber, const int _sampleSize, const Metric &_metric, const int _attempts, const int _maxIterations, const double _threshold, cv::Mat &_labels, cv::Mat &_centers)
{
	_centers = cv::Mat::zeros(_clusterNumber, _items.cols, CV_32FC1);
	_labels = cv::Mat::zeros(_items.rows, 1, CV_32FC1);

	double bestSSE = std::numeric_limits<double>::max();
	for (int i = 0; i < _attempts; i++)
	{
		std::cout << "\t** attempt " << i << std::endl;

		cv::Mat centers = cv::Mat::zeros(_clusterNumber, _items.cols, CV_32FC1);
		cv::Mat sample = cv::Mat::zeros(_sampleSize, _items.cols, CV_32FC1);
		cv::Mat sampleLabels = cv::Mat::zeros(_sampleSize, 1, CV_32FC1);

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
		cv::Mat setLabels = cv::Mat::zeros(_items.rows, 1, CV_32FC1);
		for (int k = 0; k < _items.rows; k++)
			setLabels.at<int>(k) = findClosestCenter(_items.row(k), centers, _metric);

		double sse = calculateSSE(_items, setLabels, centers, _metric);
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

bool KMeans::evaluateStopCondition(const cv::Mat &_oldCenters, const cv::Mat &_newCenters, const double _threshold, const Metric &_metric)
{
	bool thresholdReached = true;
	for (int k = 0; k < _oldCenters.rows && thresholdReached; k++)
		thresholdReached = thresholdReached && (_metric.distance(_oldCenters.row(k), _newCenters.row(k)) < _threshold);

	return thresholdReached;
}

void KMeans::selectStartCenters(const cv::Mat &_items, cv::Mat &_centers)
{
	std::vector<int> randomSet = Helper::getRandomSet(_centers.rows, 0, _items.rows);
	for (int j = 0; j < _centers.rows; j++)
		_items.row(randomSet[j]).copyTo(_centers.row(j));
}

double KMeans::calculateSSE(const cv::Mat &_items, const cv::Mat &_labels, const cv::Mat &_centers, const Metric &_metric)
{
	double sse = 0;
	for (int i = 0; i < _items.rows; i++)
	{
		double norm = _metric.distance(_items.row(i), _centers.row(_labels.at<int>(i)));
		sse += (norm * norm);
	}

	return sse;
}

int KMeans::findClosestCenter(const cv::Mat &_items, cv::Mat &_centers, const Metric &_metric)
{
	int closestCenter = -1;

	// Find the closest centroid for the current descriptor
	double minDist = std::numeric_limits<double>::max();
	for (int i = 0; i < _centers.rows; i++)
	{
		double distance = _metric.distance(_items, _centers.row(i));
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
	std::vector<int> randomSet = Helper::getRandomSet(_sample.rows, 0, _items.rows);
	for (int j = 0; j < _sample.rows; j++)
		_items.row(randomSet[j]).copyTo(_sample.row(j));
}
