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
	static void generateCube(const double _size, const pcl::PointXYZ &_center, pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud);
	static void generateCylinder(const double _radius, const double _height, const pcl::PointXYZ &_center, pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud);
	static void generateSphere(const double _radius, const pcl::PointXYZ &_center, pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud);

	static pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr createColorCloud(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const uint32_t _color);
	static pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr createColorCloud(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, uint8_t _r, uint8_t _g, uint8_t _b);
private:
	CloudFactory();
	~CloudFactory();
};
