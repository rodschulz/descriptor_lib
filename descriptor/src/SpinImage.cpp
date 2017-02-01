/**
 * Author: rodrigo
 * 2017
 */
#include "SpinImage.hpp"
#include <pcl/filters/filter.h>


void SpinImage::computeDense(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_,
							 const DescriptorParamsPtr &params_,
							 cv::Mat &descriptors_)
{
	LOGD << "Computing SpinImage dense";

	SpinImageParams *params = dynamic_cast<SpinImageParams *>(params_.get());

	// Compute the descriptor
	pcl::PointCloud<SpinImage153>::Ptr descriptorCloud(new pcl::PointCloud<SpinImage153>());

	pcl::SpinImageEstimation<pcl::PointNormal, pcl::PointNormal, SpinImage153> si;
	si.setInputCloud (cloud_);
	si.setInputNormals (cloud_);
	si.setRadiusSearch(params->searchRadius);
	si.setImageWidth(params->imageWidth);
	si.compute(*descriptorCloud);

	// Remove any NaN
	removeNaN(descriptorCloud);

	// Prepare matrix to copy data
	int rows = descriptorCloud->size();
	int cols = sizeof(SpinImage153::histogram) / sizeof(float);
	if (descriptors_.rows != rows || descriptors_.cols != cols)
		descriptors_ = cv::Mat::zeros(rows, cols, CV_32FC1);

	// Copy data to matrix
	for (int i = 0; i < rows; i++)
		memcpy(&descriptors_.at<float>(i, 0), &descriptorCloud->at(i).histogram, sizeof(float) * cols);
}

void SpinImage::computePoint(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_,
							 const DescriptorParamsPtr &params_,
							 const int target_,
							 Eigen::VectorXf &descriptor_)
{
	LOGD << "Computing SpinImage point";

	SpinImageParams *params = dynamic_cast<SpinImageParams *>(params_.get());

	// Compute the descriptor
	pcl::PointCloud<SpinImage153>::Ptr descriptorCloud(new pcl::PointCloud<SpinImage153>());

	pcl::SpinImageEstimation<pcl::PointNormal, pcl::PointNormal, SpinImage153> si;
	si.setInputCloud (cloud_);
	si.setInputNormals (cloud_);
	si.setRadiusSearch(params->searchRadius);
	si.setImageWidth(params->imageWidth);
	si.compute(*descriptorCloud);

	// Validate the descriptor
	bool isValid = true;
	size_t size = sizeof(SpinImage153::histogram) / sizeof(float);
	SpinImage153 d = descriptorCloud->at(target_);
	for (size_t i = 0; i < size && isValid; i++)
		if (!pcl_isfinite(d.histogram[i]))
			break;

	descriptor_ = Eigen::VectorXf(size);
	if (isValid)
		memcpy(descriptor_.data(), &d.histogram, sizeof(float) * size);
	else
		LOGW << "Invalid descriptor";
}

void SpinImage::removeNaN(pcl::PointCloud<SpinImage153>::Ptr &descriptors_)
{
	size_t size = sizeof(SpinImage153::histogram) / sizeof(float);
	size_t dest = 0;

	for (size_t i = 0; i < descriptors_->size(); i++)
	{
		bool remove = false;
		for (size_t j = 0; j < size; j++)
		{
			if (!pcl_isfinite((*descriptors_).points[i].histogram[j]))
			{
				remove = true;
				break;
			}
		}

		if (remove)
			continue;

		memcpy(&(*descriptors_).points[dest].histogram, &(*descriptors_).points[i].histogram, sizeof(float) * size);
		dest++;
	}

	descriptors_->resize(dest);
}
