/**
 * Author: rodrigo
 * 2016
 */
#include "CloudUtils.hpp"
#include <pcl/filters/convolution_3d.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d_omp.h>


pcl::PointCloud<pcl::PointXYZ>::Ptr CloudUtils::gaussianSmoothing(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_,
		const double sigma_,
		const double radius_)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr smoothedCloud(new pcl::PointCloud<pcl::PointXYZ>());

	//Set up the Gaussian Kernel
	pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ>::Ptr kernel(new pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ>());
	kernel->setSigma(sigma_);
	kernel->setThresholdRelativeToSigma(3);

	//Set up the KDTree
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdTree(new pcl::search::KdTree<pcl::PointXYZ>);
	kdTree->setInputCloud(cloud_);

	//Set up the Convolution Filter
	pcl::filters::Convolution3D<pcl::PointXYZ, pcl::PointXYZ, pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ> > convolution;
	convolution.setKernel(*kernel);
	convolution.setInputCloud(cloud_);
	convolution.setSearchMethod(kdTree);
	convolution.setRadiusSearch(radius_);
	convolution.convolve(*smoothedCloud);

	return smoothedCloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CloudUtils::MLSSmoothing(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_,
		const double radius_)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr smoothedCloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointNormal>::Ptr MLSPoints(new pcl::PointCloud<pcl::PointNormal>());

	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdTree(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
	mls.setComputeNormals(false);
	mls.setInputCloud(cloud_);
	mls.setPolynomialFit(true);
	mls.setSearchMethod(kdTree);
	mls.setSearchRadius(radius_);
	mls.process(*MLSPoints);

	pcl::copyPointCloud<pcl::PointNormal, pcl::PointXYZ>(*MLSPoints, *smoothedCloud);
	return smoothedCloud;
}

pcl::PointCloud<pcl::Normal>::Ptr CloudUtils::estimateNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_,
		const double searchRadius_)
{
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());

	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud(cloud_);

	if (searchRadius_ > 0)
		normalEstimation.setRadiusSearch(searchRadius_);
	else
		normalEstimation.setKSearch(10);

	normalEstimation.setSearchMethod(kdtree);
	normalEstimation.compute(*normals);

	return normals;
}

cv::Mat CloudUtils::toMatrix(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_,
							 const bool includeNormals_)
{
	cv::Mat data = cv::Mat::zeros(cloud_->size(), includeNormals_ ? 7 : 3, CV_32FC1);

	for (size_t i = 0; i < cloud_->size(); i++)
	{
		data.at<float>(i, 0) = cloud_->at(i).x;
		data.at<float>(i, 1) = cloud_->at(i).y;
		data.at<float>(i, 2) = cloud_->at(i).z;

		if (includeNormals_)
		{
			data.at<float>(i, 3) = cloud_->at(i).normal_x;
			data.at<float>(i, 4) = cloud_->at(i).normal_y;
			data.at<float>(i, 5) = cloud_->at(i).normal_z;
			data.at<float>(i, 6) = cloud_->at(i).curvature;
		}
	}

	return data;
}
