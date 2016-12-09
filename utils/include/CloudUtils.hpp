/**
 * Author: rodrigo
 * 2016
 */
#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/core/core.hpp>

class CloudUtils
{
public:
	// Remove any NAN in the given cloud
	static void removeNANs(pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud);

	// Performs a gaussian smoothing over the given cloud
	static pcl::PointCloud<pcl::PointXYZ>::Ptr gaussianSmoothing(const pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud, const double _sigma, const double _radius);

	// Performs a Min Least Squares smoothing over the given cloud
	static pcl::PointCloud<pcl::PointXYZ>::Ptr MLSSmoothing(const pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud, const double _radius);

	// Performs the estimation of normals in each point of the given cloud
	static pcl::PointCloud<pcl::Normal>::Ptr estimateNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud, const double _searchRadius = -1);

	// Translates the given point cloud to a matrix with the XYZ coordinates and optionally the normal coordinates
	static cv::Mat cloudToMatrix(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const bool _includeNormals = false);

private:
	CloudUtils();
	~CloudUtils();
};
