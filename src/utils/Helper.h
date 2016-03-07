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
#include "../factories/MetricFactory.h"

struct ExecutionParams;

class Helper
{
public:
	static void removeNANs(pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud);

	static bool loadCloud(pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const ExecutionParams &_params);
	static pcl::PointCloud<pcl::Normal>::Ptr estimateNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud, const double _searchRadius = -1);
	static pcl::PointCloud<pcl::PointXYZ>::Ptr gaussianSmoothing(const pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud, const double _sigma, const double _radius);
	static pcl::PointCloud<pcl::PointXYZ>::Ptr MLSSmoothing(const pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud, const double _radius);

	static void generateDescriptorsCache(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const ExecutionParams &_params, cv::Mat &_descriptors);

	static double calculateSSE(const cv::Mat &_descriptors, const cv::Mat &_centers, const cv::Mat &_labels);
	static void generateElbowGraph(const cv::Mat &_descriptors, const ExecutionParams &_params);
	static pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr generateClusterRepresentation(const pcl::PointCloud<pcl::PointNormal>::Ptr _cloud, const cv::Mat &_labels, const cv::Mat &_centers, const ExecutionParams &_params);

	static void evaluateMetricCases(const std::string &_resultsFilename, const std::string &_testcasesFilename, const MetricType &_metricType, const std::vector<std::string> &_args);
private:
	Helper();
	~Helper();
};
