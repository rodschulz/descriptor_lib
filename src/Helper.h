/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include "Parser.h"

using namespace std;
using namespace pcl;

// Sign function
template<typename T> int sign(T val)
{
	return (T(0) < val) - (val < T(0));
}

class Helper
{
public:
	static void removeNANs(PointCloud<PointXYZ>::Ptr &_cloud);
	static PointCloud<Normal>::Ptr getNormals(const PointCloud<PointXYZ>::Ptr &_cloud, const double _searchRadius = -1);
	static bool getCloudAndNormals(PointCloud<PointNormal>::Ptr &_cloud, const ExecutionParams &_params);

	static PointCloud<PointXYZRGB>::Ptr createColorCloud(const PointCloud<PointNormal>::Ptr &_cloud, const uint8_t _r, const uint8_t _g, const uint8_t _b);
	static float getColor(const uint8_t _r, const uint8_t _g, const uint8_t _b);

	static bool isNumber(const string &_str);

private:
	Helper();
	~Helper();
};
