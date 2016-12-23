/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <eigen3/Eigen/src/Geometry/Hyperplane.h>
#include <eigen3/Eigen/src/Geometry/ParametrizedLine.h>

class Band
{
public:
	pcl::PointCloud<pcl::PointNormal>::Ptr points; // Band's points
	pcl::PointNormal origin; // Band's origin
	Eigen::Hyperplane<float, 3> plane; // Perpendicular plane splitting the band in along it
	Eigen::ParametrizedLine<float, 3> axis; // Band's axis
	std::vector<float> descriptor; // Band's descriptor data


	/**************************************************/
	Band(const pcl::PointNormal &point_,
		 const Eigen::Hyperplane<float, 3> &plane_,
		 const Eigen::ParametrizedLine<float, 3> &axis_)
	{
		points = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>());
		origin = point_;
		plane = plane_;
		axis = axis_;
	}
};

// Declaration to define a band's shared pointer
typedef boost::shared_ptr<Band> BandPtr;
