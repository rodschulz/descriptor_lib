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
	static bool loadMatrix(const std::string &filename_, cv::Mat &matrix_, std::map<std::string, std::string> *metadata_ = NULL);

	// Loads a descriptors calculation cache. Returns true if was successfully loaded, false otherwise
	static bool loadDescriptors(const std::string &cacheLocation_, const std::string &cloudInputFilename_, const double normalEstimationRadius_, const DescriptorParams &descritorParams_, const CloudSmoothingParams &smoothingParams_, cv::Mat &descriptors_);

	// Loads pre-calculated centers for clustering. Returns true if was successfully loaded, false otherwise
	static bool loadCenters(const std::string &filename_, cv::Mat &centers_, std::map<std::string, std::string> *metadata_ = NULL);

	// Loads a matrix from disk and stores it in the given matrix. Returns true if was successfully loaded, false otherwise
	static bool loadCloud(const std::string &filename_, const double normalEstimationRadius_, const CloudSmoothingParams &params_, pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_);

	// Traverses the given directory collecting the data in the data vector
	static void traverseDirectory(const std::string &inputDirectory_, std::vector<std::pair<cv::Mat, std::map<std::string, std::string> > > &data_, std::pair<int, int> &dimensions_);

private:
	Loader();
	~Loader();
};
