/**
 * Author: rodrigo
 * 2017
 */
#pragma once

#include <pcl/features/usc.h>
#include "DescriptorParams.hpp"


class USC
{
public:
	/**************************************************/
	static void computeDense(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_,
							 const DescriptorParamsPtr &params_,
							 cv::Mat &descriptors_);

	/**************************************************/
	static void computePoint(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_,
							 const DescriptorParamsPtr &params_,
							 const int target_,
							 Eigen::VectorXf &descriptor_);

private:
	USC() {};
	~USC() {};

	/**************************************************/
	static void removeNaN(pcl::PointCloud<pcl::UniqueShapeContext1960>::Ptr &descriptors_);
};
