/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <string>
#include <stdexcept>
#include <yaml-cpp/yaml.h>
#include <yaml-cpp/node/parse.h>
#include "DescriptorParams.hpp"


#define OUTPUT_DIR				"./output/"
#define DEBUG_DIR				"./debug/"
#define CLOUD_FILE_EXTENSION	".pcd"
#define DEBUG_PREFIX			"DEBUG_"


class Config
{
public:
	~Config() {};

	/**************************************************/
	static Config *getInstance()
	{
		static Config instance = Config();
		return &instance;
	}

	/**************************************************/
	static YAML::Node get()
	{
		return getInstance()->config;
	}

	/**************************************************/
	static bool load(const std::string &filename_);

	/**************************************************/
	static bool debugEnabled()
	{
		return getInstance()->debug;
	}

	/**************************************************/
	static double getTargetPoint()
	{
		return getInstance()->targetPoint;
	}

	/**************************************************/
	static double getNormalEstimationRadius()
	{
		return getInstance()->normalEstimationRadius;
	}

	/**************************************************/
	static std::string getCacheDirectory()
	{
		return getInstance()->cacheLocation;
	}

	/**************************************************/
	static DescriptorParamsPtr getDescriptorParams()
	{
		if (!getInstance()->descriptorParams)
			throw std::runtime_error("descriptor params not loaded");

		return getInstance()->descriptorParams;
	}

	/**************************************************/
	static ClusteringParams getClusteringParams()
	{
		if (getInstance()->clusteringParams == NULL)
			throw std::runtime_error("clustering params not loaded");

		return *getInstance()->clusteringParams;
	}

	/**************************************************/
	static CloudSmoothingParams getCloudSmoothingParams()
	{
		if (getInstance()->cloudSmoothingParams == NULL)
			throw std::runtime_error("cloud smoothing params not loaded");

		return *getInstance()->cloudSmoothingParams;
	}

	/**************************************************/
	static SyntheticCloudsParams getSyntheticCloudParams()
	{
		if (getInstance()->syntheticCloudParams == NULL)
			throw std::runtime_error("synthetic cloud params not loaded");

		return *getInstance()->syntheticCloudParams;
	}


private:
	Config();

	DescriptorParamsPtr descriptorParams;
	ClusteringParams *clusteringParams;
	CloudSmoothingParams *cloudSmoothingParams;
	SyntheticCloudsParams *syntheticCloudParams;
	YAML::Node config;

	bool debug; // Flag indicating if the debug generation is enabled or not
	int targetPoint; // Target point
	double normalEstimationRadius; // Radius used to perform the normal vectors estimation
	std::string cacheLocation; // Directory where cached calculations are stored
};
