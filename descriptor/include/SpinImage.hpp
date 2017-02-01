/**
 * Author: rodrigo
 * 2017
 */
#pragma once

#include <pcl/features/pfh.h>
#include "DescriptorParams.hpp"


typedef pcl::Histogram<153> SpinImage153;

class SpinImage
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
	SpinImage() {};
	~SpinImage() {};

	/**************************************************/
	static void removeNaN(pcl::PointCloud<SpinImage153>::Ptr &descriptors_);
};
