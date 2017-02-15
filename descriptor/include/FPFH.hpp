/**
 * Author: rodrigo
 * 2017
 */
#pragma once

#include <pcl/features/fpfh.h>
#include "DescriptorParams.hpp"


#define FPFH_POINT_CPY(dest_, orig_, size_)		memcpy((dest_).data(), &(orig_).histogram, sizeof(float) * (size_))


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

	/**************************************************/
	static void removeNaN(pcl::PointCloud<pcl::FPFHSignature33>::Ptr &descriptors_);

	/**************************************************/
	static inline void copyCloud(const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &descriptorCloud_,
								 cv::Mat &descriptors_)
	{
		int rows = descriptorCloud_->size();
		int cols = sizeof(pcl::FPFHSignature33::histogram) / sizeof(float);
		if (descriptors_.rows != rows || descriptors_.cols != cols)
			descriptors_ = cv::Mat::zeros(rows, cols, CV_32FC1);

		// Copy data to matrix
		for (int i = 0; i < rows; i++)
			memcpy(&descriptors_.at<float>(i, 0), &descriptorCloud_->at(i).histogram, sizeof(float) * cols);
	}

private:
	FPFH() {};
	~FPFH() {};
};
