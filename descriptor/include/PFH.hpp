/**
 * Author: rodrigo
 * 2017
 */
#pragma once

#include <pcl/features/pfh.h>
#include "DescriptorParams.hpp"


class PFH
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
	PFH() {};
	~PFH() {};

	/**************************************************/
	static void removeNaN(pcl::PointCloud<pcl::PFHSignature125>::Ptr &descriptors_);
};
