/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <string>
#include <stdexcept>
#include "ExecutionParams.hpp"

#define OUTPUT_FOLDER			"./output/"
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

	// Loads the configuration file
	static bool load(const std::string &filename_);

	// Returns a boolean value indicating if the debug generation is enabled
	static bool debugEnabled()
	{
		return getInstance()->debug;
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

	// Returns the descritor calculation parameters
	static DescriptorParams getDescriptorParams()
	{
		if (getInstance()->descriptorParams == NULL)
			throw std::runtime_error("ERROR: descriptor params not loaded");

		return *getInstance()->descriptorParams;
	}

	// Returns the clustering parameters
	static ClusteringParams getClusteringParams()
	{
		if (getInstance()->clusteringParams == NULL)
			throw std::runtime_error("ERROR: clustering params not loaded");

		return *getInstance()->clusteringParams;
	}

	// Returns the cloud smoothing parameters
	static CloudSmoothingParams getCloudSmoothingParams()
	{
		if (getInstance()->cloudSmoothingParams == NULL)
			throw std::runtime_error("ERROR: cloud smoothing params not loaded");

		return *getInstance()->cloudSmoothingParams;
	}

private:
	// Constructor
	Config();

	// Cached config params
	DescriptorParams *descriptorParams;
	ClusteringParams *clusteringParams;
	CloudSmoothingParams *cloudSmoothingParams;
	SyntheticCloudsParams *syntheticCloudParams;
	MetricTestingParams *metricTestingParams;

	bool debug; // Flag indicating if the debug generation is enabled or not
	int targetPoint; // Target point
	double normalEstimationRadius; // Radius used to perform the normal vectors estimation
	std::string cacheLocation; // Directory where cached calculations are stored
};
