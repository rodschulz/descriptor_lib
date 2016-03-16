/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <vector>
#include "../utils/ExecutionParams.hpp"
#include "Band.hpp"

class Extractor
{
public:
	// Return the neighbors of the given point in the given cloud, according to the search radius (sphere)
	static pcl::PointCloud<pcl::PointNormal>::Ptr getNeighbors(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const pcl::PointNormal &_searchPoint, const double _searchRadius);

	// Extracts the bands around the target point and according to the given params
	static std::vector<BandPtr> getBands(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const pcl::PointNormal &_point, const ExecutionParams &_params);

	// Extracts the planes going along each band
	static std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> generatePlaneClouds(const std::vector<BandPtr> &_bands, const ExecutionParams &_params);

private:
	Extractor();
	~Extractor();
};
