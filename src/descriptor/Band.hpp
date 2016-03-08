/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <eigen3/Eigen/src/Geometry/Hyperplane.h>
#include "../factories/PointFactory.hpp"

// Forward declaration to define a band's shared pointer
class Band;
typedef boost::shared_ptr<Band> BandPtr;

class Band
{
public:
	// Vector holding the data which generated the current band
	pcl::PointCloud<pcl::PointNormal>::Ptr data;

	// Band's generator point
	pcl::PointNormal point;

	// Plane going along the longitudinal band
	Eigen::Hyperplane<float, 3> plane;

	// String representing the sequence of the current band
	std::string sequenceString;

	// Vector corresponding to the numeric sequence representation
	std::vector<float> sequenceVector;

	Band(const pcl::PointNormal &_point, const Eigen::Hyperplane<float, 3> &_plane)
	{
		data = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>());
		point = _point;
		plane = _plane;
		sequenceString = "";
	}

	Band(const pcl::PointNormal &_point)
	{
		data = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>());
		point = _point;
		plane = Eigen::Hyperplane<float, 3>(Eigen::Vector3f(0, 1, 0), Eigen::Vector3f(0, 0, 0));
		sequenceString = "";
	}

	Band()
	{
		data = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>());
		point = PointFactory::createPointNormal(1, 0, 0, 1, 0, 0);
		plane = Eigen::Hyperplane<float, 3>(Eigen::Vector3f(0, 1, 0), Eigen::Vector3f(0, 0, 0));
		sequenceString = "";
	}
};
