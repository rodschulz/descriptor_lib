/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <vector>
#include "ExecutionParams.hpp"
#include "Band.hpp"

class Extractor
{
public:
	/**************************************************/
	static pcl::PointCloud<pcl::PointNormal>::Ptr getNeighbors(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud,
			const pcl::PointNormal &searchPoint_,
			const double searchRadius_);

	/**************************************************/
	static std::vector<BandPtr> getBands(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud,
										 const pcl::PointNormal &_point,
										 const DescriptorParams &_params);

	/**************************************************/
	static std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> generatePlaneClouds(const std::vector<BandPtr> &_bands,
			const DescriptorParams &_params);

private:
	Extractor();
	~Extractor();

	/**************************************************/
	static void DEBUG_generatePointPlane(const Eigen::Hyperplane<float, 3> &_plane,
										 const Eigen::Vector3f &_p,
										 const Eigen::Vector3f &_n,
										 const float _limit,
										 const std::string &_filename,
										 const PointColor &_color);

	/**************************************************/
	static void DEBUG_generateExtractedLine(const Eigen::ParametrizedLine<float, 3> &_line,
											const float _limit,
											const std::string &_filename,
											const PointColor &_color);

	/**************************************************/
	static std::pair<float, float> DEBUG_getDebugGenerationLimit(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud);
};
