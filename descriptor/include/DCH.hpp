/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <vector>
#include "Utils.hpp"
#include "Extractor.hpp"
#include "Histogram.hpp"


class DCH
{
public:
	/**************************************************/
	static std::vector<BandPtr> calculateDescriptor(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_,
			const DescriptorParamsPtr &params_,
			const int targetPointIndex_);

	/**************************************************/
	static std::vector<BandPtr> calculateDescriptor(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_,
			const DescriptorParamsPtr &params_,
			const pcl::PointNormal &target_);

	/**************************************************/
	static void computeDense(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_,
							 const DescriptorParamsPtr &params_,
							 cv::Mat &descriptors_);

	/**************************************************/
	static void computePoint(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_,
							 const DescriptorParamsPtr &params_,
							 const int target_,
							 Eigen::VectorXf &descriptor_,
							 const std::string &debugId_ = "");

	/**************************************************/
	static std::vector<Histogram> generateAngleHistograms(const std::vector<BandPtr> &descriptor_,
			const bool useProjection_);

	/**************************************************/
	static void fillDescriptor(std::vector<BandPtr> &descriptor_,
							   const DescriptorParamsPtr &params_);

	/**************************************************/
	static inline double calculateAngle(const Eigen::Vector3f &vector1_,
										const Eigen::Vector3f &vector2_,
										const Eigen::Hyperplane<float, 3> &plane_,
										const bool useProjection_)
	{
		Eigen::Vector3f v2 = useProjection_ ? plane_.projection(vector2_).normalized() : vector2_;
		return Utils::signedAngle<Eigen::Vector3f>(vector1_, v2, (Eigen::Vector3f) plane_.normal());
	}

private:
	DCH();
	~DCH();
};

