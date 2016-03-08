/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <opencv2/core/core.hpp>

struct ExecutionParams;

class Helper
{
public:
	static void generateElbowGraph(const cv::Mat &_descriptors, const ExecutionParams &_params);
	static pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr generateClusterRepresentation(const pcl::PointCloud<pcl::PointNormal>::Ptr _cloud, const cv::Mat &_labels, const cv::Mat &_centers, const ExecutionParams &_params);
private:
	Helper();
	~Helper();
};
