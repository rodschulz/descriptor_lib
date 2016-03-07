/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <opencv2/core/core.hpp>
#include "../clustering/Metric.h"
#include "../clustering/MetricFactory.h"

struct ExecutionParams;

class Helper
{
public:
//	static bool loadCloud(pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const ExecutionParams &_params);

	static void generateDescriptorsCache(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const ExecutionParams &_params, cv::Mat &_descriptors);

	static void generateElbowGraph(const cv::Mat &_descriptors, const ExecutionParams &_params);
	static pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr generateClusterRepresentation(const pcl::PointCloud<pcl::PointNormal>::Ptr _cloud, const cv::Mat &_labels, const cv::Mat &_centers, const ExecutionParams &_params);

	static void evaluateMetricCases(const std::string &_resultsFilename, const std::string &_testcasesFilename, const MetricType &_metricType, const std::vector<std::string> &_args);
private:
	Helper();
	~Helper();
};
