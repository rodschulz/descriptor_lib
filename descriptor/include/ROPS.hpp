/**
 * Author: rodrigo
 * 2017
 */
#pragma once

#include <pcl/features/rops_estimation.h>
#include "DescriptorParams.hpp"


typedef pcl::Histogram<135> ROPS135;

class ROPS
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
	ROPS() {};
	~ROPS() {};

	/**************************************************/
	static void removeNaN(pcl::PointCloud<ROPS135>::Ptr &descriptors_);
};
