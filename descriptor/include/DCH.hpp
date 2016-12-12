/**
 * Author: rodrigo
 * 2016
 */
#pragma once

#include "DescriptorXX.hpp"
#include "ExecutionParams.hpp"


class DCH: public DescriptorXX
{
public:
	DCH()
	{
		type = DESC_DCH;
		searchRadius = 0;
		bandNumber = 4;
		bandWidth = 0.01;
		bidirectional = true;
		useProjection = true;
		binNumber = 0.01;
		sequenceStat = STAT_MEAN;
	}
	~DCH() {}

	/**************************************************/
	void load(const YAML::Node &config_);

	/**************************************************/
	std::string toString() const;

	/**************************************************/
	void computeDense(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_,
					  cv::Mat &decriptors_) const;

private:
	float searchRadius; // Extraction sphere's radius
	int bandNumber; // Number of bands to use in the descriptor
	float bandWidth; // Width of each descriptor's band
	bool bidirectional; // True if bands use the full diameter false otherwise
	bool useProjection; // True if angles are computed using a projection, false otherwise
	int binNumber; // Number of bins per band
	SequenceStat sequenceStat; // Statistic computed in each band
};
