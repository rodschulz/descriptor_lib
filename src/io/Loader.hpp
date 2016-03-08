/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/core/core.hpp>
#include <string>

#include "../utils/ExecutionParams.hpp"

class Loader
{
public:
	// Loads a matrix from disk and stores it in the given matrix. Returns true if was successfully loaded, false otherwise
	static bool loadMatrix(cv::Mat &_matrix, const std::string &_filename);

	// Loads a descriptors calculation cache. Returns true if was successfully loaded, false otherwise
	static bool loadDescriptorsCache(cv::Mat &_descriptors, const ExecutionParams &_params);

	// Loads pre-calculated centers for clustering. Returns true if was successfully loaded, false otherwise
	static bool loadClusterCenters(const std::string &_fileLocation, cv::Mat &_centers);

	// Loads a matrix from disk and stores it in the given matrix. Returns true if was successfully loaded, false otherwise
	static bool loadCloud(pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const ExecutionParams &_params);

private:

	Loader();
	~Loader();
};
