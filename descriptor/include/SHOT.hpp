/**
 * Author: rodrigo
 * 2016
 */
#pragma once

#include "DescriptorXX.hpp"


class SHOT: public DescriptorXX
{
public:
	SHOT()
	{
		type = DESC_SHOT;
		searchRadius = 0;
	}
	~SHOT() {}

	/**************************************************/
	void load(const YAML::Node &config_);

	/**************************************************/
	std::string toString() const;

	/**************************************************/
	void computeDense(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_,
					  const pcl::PointCloud<pcl::Normal>::Ptr &normals_,
					  cv::Mat &decriptors_) const;

private:
	float searchRadius; // Extraction sphere's radius
};
