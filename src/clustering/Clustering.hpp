/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/core/core.hpp>
#include "../utils/ExecutionParams.hpp"

struct ClusteringResults
{
	cv::Mat labels;
	cv::Mat centers;
	std::vector<double> errorEvolution;
};

class Clustering
{
public:
	// Performs the search of clusters according to the given parameters
	static void searchClusters(const cv::Mat &_items, const ExecutionParams &_params, ClusteringResults &_results);

	// Generates an elbow graph according to the given params (to evaluate the SSE evolution)
	static void generateElbowGraph(const cv::Mat &_items, const ExecutionParams &_params);

	// Generates a representation of the clusters
	static pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr generateClusterRepresentation(const pcl::PointCloud<pcl::PointNormal>::Ptr _cloud, const cv::Mat &_labels, const cv::Mat &_centers, const ExecutionParams &_params);

private:
	Clustering();
	~Clustering();
};
