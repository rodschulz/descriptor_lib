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

	static void DEBUG_generatePointPlane(const Eigen::Hyperplane<float, 3> &_plane, const Eigen::Vector3f &_p, const Eigen::Vector3f &_n, const float _limit);
	static void DEBUG_generateExtractedLine(const Eigen::ParametrizedLine<float, 3> &_line, const float _limit, const std::string &_filename, const PointColor &_color);
};
