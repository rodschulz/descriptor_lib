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

// Sign function
template<typename T> inline int sign(T val)
{
	return (T(0) < val) - (val < T(0));
}

class Helper
{
public:
	static int getRandomNumber(const int _min, const int _max);
	static std::vector<int> getRandomSet(const int _size, const int _min, const int _max);
	static void removeNANs(pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud);
	static std::string toHexString(const size_t _number);

	static bool loadCloud(pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const ExecutionParams &_params);
	static pcl::PointCloud<pcl::Normal>::Ptr estimateNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud, const double _searchRadius = -1);
	static pcl::PointCloud<pcl::PointXYZ>::Ptr gaussianSmoothing(const pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud, const double _sigma, const double _radius);
	static pcl::PointCloud<pcl::PointXYZ>::Ptr MLSSmoothing(const pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud, const double _radius);

	static float getColor(const uint8_t _r, const uint8_t _g, const uint8_t _b);
	static uint32_t getColor(const int _index);

	static bool loadClusteringCache(cv::Mat &_descriptors, const ExecutionParams &_params);
	static void writeClusteringCache(const cv::Mat &_descriptors, const ExecutionParams &_params);

	static double calculateSSE(const cv::Mat &_descriptors, const cv::Mat &_centers, const cv::Mat &_labels);
	static void generateElbowGraph(const cv::Mat &_descriptors, const ExecutionParams &_params);

	static pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr generateClusterRepresentation(const pcl::PointCloud<pcl::PointNormal>::Ptr _cloud, const cv::Mat &_labels, const cv::Mat &_centers, const ExecutionParams &_params);
private:
	Helper();
	~Helper();
};
