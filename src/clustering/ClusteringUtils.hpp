/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/core/core.hpp>
#include "../utils/ExecutionParams.hpp"

class ClusteringUtils
{
public:
	// Generates an elbow graph according to the given params (to evaluate the SSE evolution)
	static void generateElbowGraph(const cv::Mat &_descriptors, const ExecutionParams &_params);

	// Generates a representation of the clusters
	static pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr generateClusterRepresentation(const pcl::PointCloud<pcl::PointNormal>::Ptr _cloud, const cv::Mat &_labels, const cv::Mat &_centers, const ExecutionParams &_params);

private:
	ClusteringUtils();
	~ClusteringUtils();
};
