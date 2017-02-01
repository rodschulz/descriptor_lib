/**
 * Author: rodrigo
 * 2017
 */
#include "PFH.hpp"
#include <pcl/filters/filter.h>


void PFH::computeDense(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_,
					   const DescriptorParamsPtr &params_,
					   cv::Mat &descriptors_)
{
	LOGD << "Computing PFH dense";

	PFHParams *params = dynamic_cast<PFHParams *>(params_.get());

	// Compute the descriptor
	pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptorCloud(new pcl::PointCloud<pcl::PFHSignature125>());
	pcl::search::KdTree<pcl::PointNormal>::Ptr kdtree(new pcl::search::KdTree<pcl::PointNormal>);

	pcl::PFHEstimation<pcl::PointNormal, pcl::PointNormal, pcl::PFHSignature125> pfh;
	pfh.setInputCloud (cloud_);
	pfh.setInputNormals (cloud_);
	pfh.setSearchMethod(kdtree);
	pfh.setRadiusSearch(params->searchRadius);
	pfh.compute(*descriptorCloud);


	// Remove any NaN
	removeNaN(descriptorCloud);

	// Prepare matrix to copy data
	int rows = descriptorCloud->size();
	int cols = sizeof(pcl::PFHSignature125::histogram) / sizeof(float);
	if (descriptors_.rows != rows || descriptors_.cols != cols)
		descriptors_ = cv::Mat::zeros(rows, cols, CV_32FC1);

	// Copy data to matrix
	for (int i = 0; i < rows; i++)
		memcpy(&descriptors_.at<float>(i, 0), &descriptorCloud->at(i).histogram, sizeof(float) * cols);
}

void PFH::computePoint(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_,
					   const DescriptorParamsPtr &params_,
					   const int target_,
					   Eigen::VectorXf &descriptor_)
{
	LOGD << "Computing PFH point";

	PFHParams *params = dynamic_cast<PFHParams *>(params_.get());

	// Compute the descriptor
	pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptorCloud(new pcl::PointCloud<pcl::PFHSignature125>());
	pcl::search::KdTree<pcl::PointNormal>::Ptr kdtree(new pcl::search::KdTree<pcl::PointNormal>);

	pcl::PFHEstimation<pcl::PointNormal, pcl::PointNormal, pcl::PFHSignature125> pfh;
	pfh.setInputCloud (cloud_);
	pfh.setInputNormals (cloud_);
	pfh.setSearchMethod(kdtree);
	pfh.setRadiusSearch(params->searchRadius);
	pfh.compute(*descriptorCloud);

	// Validate the descriptor
	bool isValid = true;
	size_t size = sizeof(pcl::PFHSignature125::histogram) / sizeof(float);
	pcl::PFHSignature125 d = descriptorCloud->at(target_);
	for (size_t i = 0; i < size && isValid; i++)
		if (!pcl_isfinite(d.histogram[i]))
			break;

	descriptor_ = Eigen::VectorXf(size);
	if (isValid)
		memcpy(descriptor_.data(), &d.histogram, sizeof(float) * size);
	else
		LOGW << "Invalid descriptor";
}

void PFH::removeNaN(pcl::PointCloud<pcl::PFHSignature125>::Ptr &descriptors_)
{
	size_t size = sizeof(pcl::PFHSignature125::histogram) / sizeof(float);
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
