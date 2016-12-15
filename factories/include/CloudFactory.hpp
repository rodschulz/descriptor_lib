/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class CloudFactory
{
public:
	// Creates a cloud shaped as a cube
	static pcl::PointCloud<pcl::PointNormal>::Ptr createCube(const double size_, const Eigen::Vector3f &_center, const int _npoints);

	// Creates a cloud shaped as a cylinder
	static pcl::PointCloud<pcl::PointNormal>::Ptr createCylinderSection(const float _angle, const float _radius, const float _height, const Eigen::Vector3f &_center, const int _npoints);

	// Creates a cloud shaped as a plane
	static pcl::PointCloud<pcl::PointNormal>::Ptr createHorizontalPlane(const float _minX, const float _maxX, const float _minY, const float _maxY, const float _z, const int _npoints);

	// Creates a cloud holding a section of a sphere according to the given azimuth
	static pcl::PointCloud<pcl::PointNormal>::Ptr createSphereSection(const float _azimuth, const float _radius, const Eigen::Vector3f &_center, const int _npoints);

	// Creates a cloud colored with the given color
	static pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr createColorCloud(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_, uint32_t color_);

	// Creates a cloud colored with the given color
	static pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr createColorCloud(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_, uint8_t r_, uint8_t g_, uint8_t b_);
private:
	CloudFactory();
	~CloudFactory();
};
