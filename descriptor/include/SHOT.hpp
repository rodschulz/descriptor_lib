/**
 * Author: rodrigo
 * 2016
 */
#pragma once

#include <iostream>
#include <pcl/features/shot.h>
#include <pcl/filters/filter.h>
#include "DescriptorParams.hpp"


class SHOT
{
public:
	static void computeDense(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_,
							 const DescriptorParamsPtr &params_,
							 cv::Mat &descriptors_)
	{
		SHOTParams *params = dynamic_cast<SHOTParams *>(params_.get());

		// Compute the descriptor
		pcl::PointCloud<pcl::SHOT352>::Ptr descriptorCloud(new pcl::PointCloud<pcl::SHOT352>());
		pcl::SHOTEstimation<pcl::PointNormal, pcl::PointNormal, pcl::SHOT352> shot;
		shot.setInputCloud(cloud_);
		shot.setInputNormals(cloud_);
		shot.setRadiusSearch(params->searchRadius);
		shot.setLRFRadius(params->searchRadius);
		shot.compute(*descriptorCloud);

		// Remove any NaN
		removeNaN(descriptorCloud);

		// Prepare matrix to copy data
		int rows = descriptorCloud->size();
		int cols = sizeof(pcl::SHOT352::descriptor) / sizeof(float);
		if (descriptors_.rows != rows || descriptors_.cols != cols)
			descriptors_ = cv::Mat::zeros(rows, cols, CV_32FC1);

		// Copy data to matrix
		for (int i = 0; i < rows; i++)
			memcpy(&descriptors_.at<float>(i, 0), &descriptorCloud->at(i).descriptor, sizeof(float) * cols);
	}

private:
	static void removeNaN(pcl::PointCloud<pcl::SHOT352>::Ptr &descriptors_)
	{
		size_t size = sizeof(pcl::SHOT352::descriptor) / sizeof(float);
		size_t dest = 0;

		for (size_t i = 0; i < descriptors_->size(); i++)
		{
			bool remove = false;
			for (size_t j = 0; j < size; j++)
			{
				if (!pcl_isfinite((*descriptors_).points[i].descriptor[j]))
				{
					remove = true;
					break;
				}
			}

			if (remove)
				continue;

			memcpy(&(*descriptors_).points[dest].descriptor, &(*descriptors_).points[i].descriptor, sizeof(float) * size);
			dest++;
		}

		descriptors_->resize(dest);
	}
};
