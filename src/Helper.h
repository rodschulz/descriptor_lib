/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;
using namespace pcl;

class Helper
{
public:
	static void removeNANs(PointCloud<PointXYZ>::Ptr &_cloud);
	static void createColorCloud(const PointCloud<PointXYZ>::Ptr &_cloud, PointCloud<PointXYZRGB>::Ptr &_coloredCloud, const uint8_t _r, const uint8_t _g, const uint8_t _b);
	static float getColor(const uint8_t _r, const uint8_t _g, const uint8_t _b);

private:
	Helper();
	~Helper();
};
