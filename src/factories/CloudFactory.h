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
	static pcl::PointCloud<pcl::PointXYZ>::Ptr createCube(const double _size, const pcl::PointXYZ &_center);

	// Creates a cloud shaped as a cylinder
	static pcl::PointCloud<pcl::PointXYZ>::Ptr createCylinder(const double _radius, const double _height, const pcl::PointXYZ &_center);

	// Creates a cloud shaped as a sphere
	static pcl::PointCloud<pcl::PointXYZ>::Ptr createSphere(const double _radius, const pcl::PointXYZ &_center);

	// Creates a cloud colored with the given color
	static pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr createColorCloud(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const uint32_t _color);

	// Creates a cloud colored with the given color
	static pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr createColorCloud(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, uint8_t _r, uint8_t _g, uint8_t _b);
private:
	CloudFactory();
	~CloudFactory();
};
