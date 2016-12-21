/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <vector>
#include "DescriptorParams.hpp"
#include "Band.hpp"
#include "Utils.hpp"

class Extractor
{
public:
	/**************************************************/
	static pcl::PointCloud<pcl::PointNormal>::Ptr getNeighbors(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_,
			const pcl::PointNormal &searchPoint_,
			const double searchRadius_);

	/**************************************************/
	static std::vector<BandPtr> getBands(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_,
										 const pcl::PointNormal &point_,
										 const DCHParams *params_);

	/**************************************************/
	static std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> generatePlaneClouds(const std::vector<BandPtr> &bands_,
			const DCHParams *params_);

private:
	Extractor();
	~Extractor();

	/**************************************************/
	static void DEBUG_generatePointPlane(const Eigen::Hyperplane<float, 3> &plane_,
										 const Eigen::Vector3f &p_,
										 const Eigen::Vector3f &n_,
										 const float limit_,
										 const std::string &filename_,
										 const PointColor &color_);

	/**************************************************/
	static void DEBUG_generateExtractedLine(const Eigen::ParametrizedLine<float, 3> &line_,
											const float limit_,
											const std::string &filename_,
											const PointColor &color_);

	/**************************************************/
	static std::pair<float, float> DEBUG_getDebugGenerationLimit(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_);
};
