/**
 * Author: rodrigo
 * 2017
 */
#pragma once

#include <pcl/features/fpfh.h>
#include "DescriptorParams.hpp"


class FPFH
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
	FPFH() {};
	~FPFH() {};

	/**************************************************/
	static void removeNaN(pcl::PointCloud<pcl::FPFHSignature33>::Ptr &descriptors_);
};
