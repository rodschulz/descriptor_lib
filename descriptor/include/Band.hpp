/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <eigen3/Eigen/src/Geometry/Hyperplane.h>

class Band
{
public:
	pcl::PointCloud<pcl::PointNormal>::Ptr data; // Band's points
	pcl::PointNormal point; // Band's origin
	Eigen::Hyperplane<float, 3> plane; // Plane perpendicular which splits the band in 2 along it
	std::vector<float> sequenceVector; // Band's descriptor vector


	/**************************************************/
	Band(const pcl::PointNormal &point_,
		 const Eigen::Hyperplane<float, 3> &plane_)
	{
		data = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>());
		point = point_;
		plane = plane_;
	}

	/**************************************************/
	Band(const pcl::PointNormal &point_)
	{
		data = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>());
		point = point_;
		plane = Eigen::Hyperplane<float, 3>(Eigen::Vector3f(0, 1, 0), Eigen::Vector3f(0, 0, 0));
	}
};

// Declaration to define a band's shared pointer
typedef boost::shared_ptr<Band> BandPtr;
