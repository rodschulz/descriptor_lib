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

private:
	CloudFactory();
	~CloudFactory();
};
