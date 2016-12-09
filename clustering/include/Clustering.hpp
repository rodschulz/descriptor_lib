/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/core/core.hpp>
#include "ExecutionParams.hpp"
#include "Metric.hpp"

// Struct grouping the results given by the clustering algorithm
struct ClusteringResults
{
	cv::Mat labels;
	cv::Mat centers;
	std::vector<double> errorEvolution;

	// Prepares the current structure to store new results
	void prepare(const int _ncluster, const int _nitems, const int _dim)
	{
		centers = cv::Mat::zeros(_ncluster, _dim, CV_32FC1);
		labels = cv::Mat::zeros(_nitems, 1, CV_32SC1);
		errorEvolution.clear();
	}
};

// Clustering class definition
class Clustering
{
public:
	// Performs the search of clusters according to the given parameters
	static void searchClusters(const cv::Mat &items_, const ClusteringParams &params_, ClusteringResults &_results);

	// Generates an elbow graph according to the given params (to evaluate the SSE evolution)
	static void generateElbowGraph(const cv::Mat &items_, const ClusteringParams &params_);

	// Generates a representation of the clusters
	static pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr generateClusterRepresentation(const pcl::PointCloud<pcl::PointNormal>::Ptr cloud_, const cv::Mat &labels_, const cv::Mat &centers_, const DescriptorParams &params_);


	/** Auxiliary methods for useful data generation */

	// Generates a point to point distance matrix
	static cv::Mat generatePointDistanceMatrix(const cv::Mat &items_, const MetricPtr &metric_);
	static cv::Mat generatePointDistanceMatrix2(const cv::Mat &items_, const MetricPtr &metric_);
	static void generatePointDistanceMatrix3(cv::Mat &_distanceMatrix, const cv::Mat &items_, const MetricPtr &metric_);

private:
	Clustering();
	~Clustering();
};
