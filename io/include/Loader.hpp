/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/core/core.hpp>
#include <string>
#include "ExecutionParams.hpp"

class Loader
{
public:
	// Loads a matrix from disk and stores it in the given matrix. Returns true if was successfully loaded, false otherwise
	static bool loadMatrix(cv::Mat &_matrix, const std::string &_filename);

	// Loads a descriptors calculation cache. Returns true if was successfully loaded, false otherwise
	static bool loadDescriptors(const std::string &_cacheLocation, const std::string &_cloudInputFilename, const double _normalEstimationRadius, const DescriptorParams &_descritorParams, const CloudSmoothingParams &_smoothingParams, cv::Mat &_descriptors);

	// Loads pre-calculated centers for clustering. Returns true if was successfully loaded, false otherwise
	static bool loadCenters(const std::string &_fileLocation, cv::Mat &_centers);

	// Loads a matrix from disk and stores it in the given matrix. Returns true if was successfully loaded, false otherwise
	static bool loadCloud(const std::string &_filename, const double _normalEstimationRadius, const CloudSmoothingParams &_params, pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud);

private:
	Loader();
	~Loader();
};
