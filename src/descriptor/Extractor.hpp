/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <vector>

#include "../utils/ExecutionParams.hpp"
#include "Band.h"

class Extractor
{
public:
	static pcl::PointCloud<pcl::PointNormal>::Ptr getNeighbors(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const pcl::PointNormal &_searchPoint, const double _searchRadius);
	static std::vector<BandPtr> getBands(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const pcl::PointNormal &_point, const ExecutionParams &_params);
	static pcl::PointCloud<pcl::PointXYZRGB>::Ptr getTangentPlane(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const pcl::PointNormal &_point);
	static std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> getBandPlanes(const std::vector<BandPtr> &_bands, const ExecutionParams &_params);

private:
	Extractor();
	~Extractor();

	static std::vector<BandPtr> getLongitudinalBands(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const pcl::PointNormal &_point, const ExecutionParams &_params);
};
