/**
 * Author: rodrigo
 * 2016
 */
#include "CloudUtils.hpp"
#include <pcl/filters/filter.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d_omp.h>

void CloudUtils::removeNANs(pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud)
{
	std::vector<int> mapping;
	pcl::removeNaNFromPointCloud(*_cloud, *_cloud, mapping);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CloudUtils::gaussianSmoothing(const pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud, const double _sigma, const double _radius)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr smoothedCloud(new pcl::PointCloud<pcl::PointXYZ>());

	//Set up the Gaussian Kernel
	pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ>::Ptr kernel(new pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ>());
	kernel->setSigma(_sigma);
	kernel->setThresholdRelativeToSigma(3);

	//Set up the KDTree
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdTree(new pcl::search::KdTree<pcl::PointXYZ>);
	kdTree->setInputCloud(_cloud);

	//Set up the Convolution Filter
	pcl::filters::Convolution3D<pcl::PointXYZ, pcl::PointXYZ, pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ> > convolution;
	convolution.setKernel(*kernel);
	convolution.setInputCloud(_cloud);
	convolution.setSearchMethod(kdTree);
	convolution.setRadiusSearch(_radius);
	convolution.convolve(*smoothedCloud);

	return smoothedCloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CloudUtils::MLSSmoothing(const pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud, const double _radius)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr smoothedCloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointNormal>::Ptr MLSPoints(new pcl::PointCloud<pcl::PointNormal>());

	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdTree(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
	mls.setComputeNormals(false);
	mls.setInputCloud(_cloud);
	mls.setPolynomialFit(true);
	mls.setSearchMethod(kdTree);
	mls.setSearchRadius(_radius);
	mls.process(*MLSPoints);

	pcl::copyPointCloud<pcl::PointNormal, pcl::PointXYZ>(*MLSPoints, *smoothedCloud);
	return smoothedCloud;
}

pcl::PointCloud<pcl::Normal>::Ptr CloudUtils::estimateNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud, const double _searchRadius)
{
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());

	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud(_cloud);

	if (_searchRadius > 0)
		normalEstimation.setRadiusSearch(_searchRadius);
	else
		normalEstimation.setKSearch(10);

	normalEstimation.setSearchMethod(kdtree);
	normalEstimation.compute(*normals);

	return normals;
}
