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
template<typename T> inline int sign(T val)
{
	return (T(0) < val) - (val < T(0));
}

class Helper
{
public:
	static void removeNANs(PointCloud<PointXYZ>::Ptr &_cloud);
	static PointCloud<Normal>::Ptr getNormals(const PointCloud<PointXYZ>::Ptr &_cloud, const double _searchRadius = -1);
	static bool getCloud(PointCloud<PointNormal>::Ptr &_cloud, const ExecutionParams &_params);
	static PointCloud<PointXYZ>::Ptr gaussianSmoothing(const PointCloud<PointXYZ>::Ptr &_cloud, const double _sigma, const double _radius);
	static PointCloud<PointXYZ>::Ptr MLSSmoothing(const PointCloud<PointXYZ>::Ptr &_cloud, const double _radius);

	static PointCloud<PointXYZRGBNormal>::Ptr createColorCloud(const PointCloud<PointNormal>::Ptr &_cloud, const uint32_t _color);
	static PointCloud<PointXYZRGBNormal>::Ptr createColorCloud(const PointCloud<PointNormal>::Ptr &_cloud, uint8_t _r, uint8_t _g, uint8_t _b);
	static float getColor(const uint8_t _r, const uint8_t _g, const uint8_t _b);
	static uint32_t getColor(const int _index);

	static bool isNumber(const string &_str);

private:
	Helper();
	~Helper();
};
