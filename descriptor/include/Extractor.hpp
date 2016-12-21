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
	static pcl::PointCloud<pcl::PointNormal>::Ptr
	getNeighbors(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_,
				 const pcl::PointNormal &searchPoint_,
				 const double searchRadius_);

	/**************************************************/
	static std::vector<BandPtr> getBands(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_,
										 const pcl::PointNormal &point_,
										 const DCHParams *params_);

	/**************************************************/
	static std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr>
	generatePlaneClouds(const std::vector<BandPtr> &bands_,
						const DCHParams *params_);

private:
	Extractor();
	~Extractor();

	/**************************************************/
	static std::pair<Eigen::Vector3f, Eigen::Vector3f>
	generateAxes(const Eigen::Vector3f point_,
				 const Eigen::Vector3f normal_,
				 const Eigen::Hyperplane<float, 3> plane_,
				 const float angle_);

	/**************************************************/
	static void DEBUG_genPlane(const Eigen::Hyperplane<float, 3> &plane_,
							   const Eigen::Vector3f &p_,
							   const Eigen::Vector3f &n_,
							   const float limit_,
							   const std::string &filename_,
							   const PointColor &color_);

	/**************************************************/
	static void DEBUG_genLine(const Eigen::ParametrizedLine<float, 3> &line_,
							  const float limit_,
							  const std::string &filename_,
							  const PointColor &color_,
							  const bool full_ = true);

	/**************************************************/
	static std::pair<float, float> DEBUG_getLimits(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_);
};
