/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <opencv2/core/core.hpp>
#include <string>
#include "../utils/ExecutionParams.h"

class Loader
{
public:
	static bool loadDescriptorsCache(cv::Mat &_descriptors, const ExecutionParams &_params);
	static bool loadClusterCenters(const std::string &_fileLocation, cv::Mat &_centers);

private:
	static bool loadMatrix(cv::Mat &_matrix, const std::string &_filename);

	Loader();
	~Loader();
};
