/**
 * Author: rodrigo
 * 2015
 */
#include "Clustering.hpp"
#include <string.h>
#include <fstream>
#include "Utils.hpp"
#include "PointFactory.hpp"
#include "KMeans.hpp"
#include "ClusteringUtils.hpp"


void Clustering::searchClusters(const cv::Mat &items_,
								const ClusteringParams &params_,
								ClusteringResults &results_)
{
	switch (params_.implementation)
	{
	default:
		std::cout << "WARNING: invalid clustering method. Falling back to OpenCV" << std::endl;

	case CLUSTERING_OPENCV:
		cv::kmeans(items_, params_.clusterNumber, results_.labels, cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, params_.maxIterations, params_.stopThreshold), params_.attempts, cv::KMEANS_PP_CENTERS, results_.centers);
		break;

	case CLUSTERING_KMEANS:
		KMeans::searchClusters(results_, items_, params_.metric, params_.clusterNumber, params_.attempts, params_.maxIterations, params_.stopThreshold);
		break;

	case CLUSTERING_STOCHASTIC:
		KMeans::stochasticSearchClusters(results_, items_, params_.metric, params_.clusterNumber, params_.attempts, params_.maxIterations, params_.stopThreshold, items_.rows / 10);
		break;
	}
}

void Clustering::generateElbowGraph(const cv::Mat &items_,
									const ClusteringParams &params_)
{
	std::cout << "*** ELBOW: begining graph generation ***" << std::endl;

	// Clustering params for the graph generation
	ClusteringParams params = params_;
	params.attempts = 3;

	// Iterate clustering from 2 to 50 centers
	std::ofstream data;
	data.open("./output/elbow.dat", std::fstream::out);
	for (int i = 2; i < 50; i++)
	{
		std::cout << "*** ELBOW: clustering " << i << " centers" << std::endl;

		ClusteringResults results;
		searchClusters(items_, params_, results);

		double sse;
		if (params.implementation == CLUSTERING_OPENCV)
			sse = Utils::getSSE(items_, results.centers, results.labels);
		else
			sse = ClusteringUtils::getSSE(items_, results.labels, results.centers, params_.metric);

		data << i << " " << sse << "\n";
	}
	data.close();

	std::ofstream graph;
	graph.open("./output/elbow.plot", std::fstream::out);

	graph << "set xtic auto\n";
	graph << "set ytic auto\n";
	graph << "set grid ytics xtics\n\n";

	graph << "set title 'Sum of Squared Errors'\n";
	graph << "set xlabel 'Number of Clusters'\n";
	graph << "set ylabel 'SSE'\n\n";

	graph << "set term png\n";
	graph << "set output './output/elbow.png'\n\n";

	graph << "plot './output/elbow.dat' using 1:2 title 'SSE' with linespoints lt rgb 'blue'\n";

	graph.close();

	if (system("gnuplot ./output/graph.plot") != 0)
		std::cout << "WARNING: can't execute GNUPlot" << std::endl;

	std::cout << "*** ELBOW: graph generation finished ***" << std::endl;
}

cv::Mat Clustering::generatePointDistanceMatrix(const cv::Mat &items_,
		const MetricPtr &metric_)
{
	cv::Mat distanceMatrix = cv::Mat::zeros(items_.rows, items_.rows, CV_32FC1);

	for (int i = 0; i < items_.rows; i++)
	{
		for (int j = 0; j <= i; j++)
		{
			float distance = (float) metric_->distance(items_.row(i), items_.row(j));
			distanceMatrix.at<float>(i, j) = distance;
			distanceMatrix.at<float>(j, i) = distance;
		}
	}

	return distanceMatrix;
}

cv::Mat Clustering::generatePointDistanceMatrix2(const cv::Mat &items_,
		const MetricPtr &metric_)
{
	cv::Mat distanceMatrix = cv::Mat::zeros(items_.rows, items_.rows, CV_32FC1);

	#pragma omp parallel for shared(distanceMatrix)
	for (int i = 0; i < items_.rows; i++)
	{
		for (int j = 0; j <= i; j++)
		{
			float distance = (float) metric_->distance(items_.row(i), items_.row(j));
			distanceMatrix.at<float>(i, j) = distance;
			distanceMatrix.at<float>(j, i) = distance;
		}
	}

	return distanceMatrix;
}

void Clustering::generatePointDistanceMatrix3(cv::Mat &distanceMatrix_,
		const cv::Mat &items_,
		const MetricPtr &metric_)
{
	distanceMatrix_ = cv::Mat::zeros(items_.rows, items_.rows, CV_32FC1);

	for (int i = 0; i < items_.rows; i++)
	{
		for (int j = 0; j <= i; j++)
		{
			float distance = (float) metric_->distance(items_.row(i), items_.row(j));
			distanceMatrix_.at<float>(i, j) = distance;
			distanceMatrix_.at<float>(j, i) = distance;
		}
	}
}
