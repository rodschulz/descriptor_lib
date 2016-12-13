/**
 * Author: rodrigo
 * 2016
 */
#include "SHOT.hpp"
#include <pcl/features/shot.h>
#include <pcl/common/io.h>


void SHOT::load(const YAML::Node &config_)
{
	searchRadius = config_["searchRadius"].as<float>();
}

std::string SHOT::toString() const
{
	std::stringstream stream;
	stream << "searchRadius:" << searchRadius;
	return stream.str();
}

void SHOT::computeDense(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_,
						const pcl::PointCloud<pcl::Normal>::Ptr &normals_,
						cv::Mat &descriptors_) const
{
	// Compute the descriptor
	pcl::PointCloud<pcl::SHOT352>::Ptr descCloud(new pcl::PointCloud<pcl::SHOT352>());
	pcl::SHOTEstimation<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> shot;
	shot.setInputCloud(cloud_);
	shot.setInputNormals(normals_);
	shot.setRadiusSearch(searchRadius);
	shot.setLRFRadius(searchRadius);
	shot.compute(*descCloud);

	// Prepare matrix to copy data
	int rows = descCloud->size();
	int cols = sizeof(pcl::SHOT352().descriptor) / sizeof(float);
	if (descriptors_.rows != rows || descriptors_.cols != cols)
		descriptors_ = cv::Mat::zeros(rows, cols, CV_32FC1);

	// Copy data to matrix
	for (int i = 0; i < rows; i++)
		memcpy(&descriptors_.at<float>(i), &descCloud->at(i).descriptor, cols);
}
