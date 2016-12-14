/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <string>
#include <stdexcept>
#include "ExecutionParams.hpp"
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
	// Destructor
	~Config();

	// Returns the instance of the singleton
	static Config *getInstance()
	{
		static Config instance = Config();
		return &instance;
	}

	// Returns the config root node
	static YAML::Node get()
	{
		return getInstance()->config;
	}

	// Loads the configuration file
	static bool load(const std::string &filename_);

	// Returns a boolean value indicating if the debug generation is enabled
	static bool debugEnabled()
	{
		return getInstance()->debug;
	}

	// Returns the target point for descriptor evaluation
	static double getTargetPoint()
	{
		return getInstance()->targetPoint;
	}

	// Returns the normal estimation radius
	static double getNormalEstimationRadius()
	{
		return getInstance()->normalEstimationRadius;
	}

	// Returns the normal estimation radius
	static std::string getCacheDirectory()
	{
		return getInstance()->cacheLocation;
	}

	// Returns the descriptor calculation parameters
	static DescriptorParamsPtr getDescriptorParams()
	{
		if (!getInstance()->descriptorParams)
			throw std::runtime_error("descriptor params not loaded");

		return getInstance()->descriptorParams;
	}

	// Returns the clustering parameters
	static ClusteringParams getClusteringParams()
	{
		if (getInstance()->clusteringParams == NULL)
			throw std::runtime_error("clustering params not loaded");

		return *getInstance()->clusteringParams;
	}

	// Returns the cloud smoothing parameters
	static CloudSmoothingParams getCloudSmoothingParams()
	{
		if (getInstance()->cloudSmoothingParams == NULL)
			throw std::runtime_error("cloud smoothing params not loaded");

		return *getInstance()->cloudSmoothingParams;
	}

	// Returns the cloud smoothing parameters
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
