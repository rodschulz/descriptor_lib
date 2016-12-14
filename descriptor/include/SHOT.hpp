/**
 * Author: rodrigo
 * 2016
 */
#pragma once

#include <iostream>
#include <pcl/features/shot.h>
#include "ExecutionParams.hpp"


class SHOT
{
public:
	static void computeDense(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_,
							 const DescriptorParams &params_,
							 cv::Mat &descriptors_)
	{
		// Compute the descriptor
		pcl::PointCloud<pcl::SHOT352>::Ptr descriptorCloud(new pcl::PointCloud<pcl::SHOT352>());
		pcl::SHOTEstimation<pcl::PointNormal, pcl::PointNormal, pcl::SHOT352> shot;
		shot.setInputCloud(cloud_);
		shot.setInputNormals(cloud_);
		shot.setRadiusSearch(params_.searchRadius);
		shot.setLRFRadius(params_.searchRadius);
		shot.compute(*descriptorCloud);

		// Prepare matrix to copy data
		int rows = descriptorCloud->size();
		int cols = sizeof(pcl::SHOT352::descriptor) / sizeof(float);
		if (descriptors_.rows != rows || descriptors_.cols != cols)
			descriptors_ = cv::Mat::zeros(rows, cols, CV_32FC1);

		// Copy data to matrix
		for (int i = 0; i < rows; i++)
			memcpy(&descriptors_.at<float>(i, 0), &descriptorCloud->at(i).descriptor, sizeof(float) * cols);
	}
};
