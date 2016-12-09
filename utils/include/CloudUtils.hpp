/**
 * Author: rodrigo
 * 2016
 */
#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <opencv2/core/core.hpp>

class CloudUtils
{
public:
	/**************************************************/
	static void removeNANs(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_)
	{
		std::vector<int> mapping;
		pcl::removeNaNFromPointCloud(*cloud_, *cloud_, mapping);
	}

	/**************************************************/
	static pcl::PointCloud<pcl::PointXYZ>::Ptr gaussianSmoothing(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_,
			const double sigma_,
			const double radius_);

	/**************************************************/
	static pcl::PointCloud<pcl::PointXYZ>::Ptr MLSSmoothing(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_,
			const double radius_);

	/**************************************************/
	static pcl::PointCloud<pcl::Normal>::Ptr estimateNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_,
			const double searchRadius_ = -1);

	/**************************************************/
	static cv::Mat toMatrix(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_,
							const bool includeNormals_ = false);

private:
	CloudUtils();
	~CloudUtils();
};
