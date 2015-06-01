/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace pcl;

class CloudFactory
{
public:
	static void generateCube(const double _size, const PointXYZ &_center, PointCloud<PointXYZ>::Ptr &_cloud);
	static void generateCylinder(const double _radius, const double _height, const PointXYZ &_center, PointCloud<PointXYZ>::Ptr &_cloud);
	static void generateSphere(const double _radius, const PointXYZ &_center, PointCloud<PointXYZ>::Ptr &_cloud);

private:
	CloudFactory();
	~CloudFactory();
};
