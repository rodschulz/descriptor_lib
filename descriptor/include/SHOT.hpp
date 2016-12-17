/**
 * Author: rodrigo
 * 2016
 */
#pragma once

#include <pcl/features/shot.h>
#include "DescriptorParams.hpp"


class SHOT
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
	SHOT() {};
	~SHOT() {};

	/**************************************************/
	static void removeNaN(pcl::PointCloud<pcl::SHOT352>::Ptr &descriptors_);
};
