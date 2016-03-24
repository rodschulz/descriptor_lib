/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/core/core.hpp>
#include "../utils/ExecutionParams.hpp"

// Struct grouping the results given by the clustering algorithm
struct ClusteringResults
{
	cv::Mat labels;
	cv::Mat centers;
	std::vector<double> errorEvolution;

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
	static void searchClusters(const cv::Mat &_items, const ExecutionParams &_params, ClusteringResults &_results);

	// Labels the given data using the given centers
	static void labelData(const cv::Mat &_items, const cv::Mat &_centers, const ExecutionParams &_params, cv::Mat &_labels);

	// Generates an elbow graph according to the given params (to evaluate the SSE evolution)
	static void generateElbowGraph(const cv::Mat &_items, const ExecutionParams &_params);

	// Generates a representation of the clusters
	static pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr generateClusterRepresentation(const pcl::PointCloud<pcl::PointNormal>::Ptr _cloud, const cv::Mat &_labels, const cv::Mat &_centers, const ExecutionParams &_params);

private:
	Clustering();
	~Clustering();
};
