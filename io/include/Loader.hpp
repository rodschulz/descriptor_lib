/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/core/core.hpp>
#include <string>
#include "DescriptorXX.hpp"
#include "ExecutionParams.hpp"

class Loader
{
public:
	/**************************************************/
	static bool loadMatrix(const std::string &filename_,
						   cv::Mat &matrix_,
						   std::map<std::string, std::string> *metadata_ = NULL);

	/**************************************************/
	static bool loadDescriptors(const std::string &cacheDir_,
								const std::string &cloudInputFilename_,
								const double normalEstimationRadius_,
								const DescriptorPtr &descritor_,
								const CloudSmoothingParams &smoothingParams_,
								cv::Mat &descriptors_);

	/**************************************************/
	static bool loadCloud(const std::string &filename_,
						  const CloudSmoothingParams &params_,
						  pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_);

	/**************************************************/
	static void traverseDirectory(const std::string &inputDirectory_,
								  std::vector<std::pair<cv::Mat, std::map<std::string, std::string> > > &data_,
								  std::pair<int, int> &dimensions_);

private:
	Loader();
	~Loader();
};
