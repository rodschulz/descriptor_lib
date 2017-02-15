/**
 * Author: rodrigo
 * 2017
 */
#include "FPFH.hpp"
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>


void FPFH::computeDense(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_,
						const DescriptorParamsPtr &params_,
						cv::Mat &descriptors_)
{
	LOGD << "Computing FPFH dense";

	FPFHParams *params = dynamic_cast<FPFHParams *>(params_.get());

	// Compute the descriptor
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptorCloud(new pcl::PointCloud<pcl::FPFHSignature33>());
	pcl::search::KdTree<pcl::PointNormal>::Ptr kdtree(new pcl::search::KdTree<pcl::PointNormal>);

	pcl::FPFHEstimation<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> fpfh;
	fpfh.setInputCloud (cloud_);
	fpfh.setInputNormals (cloud_);
	fpfh.setSearchMethod(kdtree);
	fpfh.setRadiusSearch(params->searchRadius);
	fpfh.compute(*descriptorCloud);

	// Remove any NaN
	FPFH::removeNaN(descriptorCloud);

	// Copy data to matrix
	FPFH::copyCloud(descriptorCloud, descriptors_);
}

void FPFH::computePoint(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_,
						const DescriptorParamsPtr &params_,
						const int target_,
						Eigen::VectorXf &descriptor_)
{
	LOGD << "Computing FPFH point";

	FPFHParams *params = dynamic_cast<FPFHParams *>(params_.get());


	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimation<pcl::PointNormal, pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud(cloud_);
	normalEstimation.setRadiusSearch(0.03);
	pcl::search::KdTree<pcl::PointNormal>::Ptr kdtreeN(new pcl::search::KdTree<pcl::PointNormal>);
	normalEstimation.setSearchMethod(kdtreeN);
	normalEstimation.compute(*normals);



	// Compute the descriptor
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptorCloud(new pcl::PointCloud<pcl::FPFHSignature33>());
	pcl::search::KdTree<pcl::PointNormal>::Ptr kdtree(new pcl::search::KdTree<pcl::PointNormal>);

	// pcl::FPFHEstimation<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> fpfh;
	pcl::FPFHEstimation<pcl::PointNormal, pcl::Normal, pcl::FPFHSignature33> fpfh;
	fpfh.setInputCloud (cloud_);
	fpfh.setInputNormals (normals);
	fpfh.setSearchMethod(kdtree);
	fpfh.setRadiusSearch(params->searchRadius);
	fpfh.compute(*descriptorCloud);

	// Validate the descriptor
	bool isValid = true;
	size_t size = sizeof(pcl::FPFHSignature33::histogram) / sizeof(float);
	pcl::FPFHSignature33 d = descriptorCloud->at(target_);
	for (size_t i = 0; i < size && isValid; i++)
		if (!pcl_isfinite(d.histogram[i]))
			break;

	descriptor_ = Eigen::VectorXf(size);
	if (isValid)
		FPFH_POINT_CPY(descriptor_, d, size);
	else
		LOGW << "Invalid descriptor";
}

void FPFH::removeNaN(pcl::PointCloud<pcl::FPFHSignature33>::Ptr &descriptors_)
{
	size_t size = sizeof(pcl::FPFHSignature33::histogram) / sizeof(float);
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
