/**
 * Author: rodrigo
 * 2015
 */
#include "Config.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <iterator>
#include <math.h>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <yaml-cpp/yaml.h>
#include <yaml-cpp/node/parse.h>

Config::Config()
{
	debug = false;
	targetPoint = -1;
	normalEstimationRadius = -1;

	descriptorParams = NULL;
	clusteringParams = NULL;
	cloudSmoothingParams = NULL;
	syntheticCloudParams = NULL;
	metricTestingParams = NULL;
}

Config::~Config()
{
}

bool Config::load(const std::string &filename_)
{
	bool loadOk = true;
	try
	{
		YAML::Node config = YAML::LoadFile(filename_);

		getInstance()->debug = config["debug"].as<bool>();
		getInstance()->targetPoint = config["targetPint"] ? config["targetPint"].as<bool>() : -1;
		getInstance()->normalEstimationRadius = config["normalRadius"] ? config["normalEstimationRadius"].as<double>() : -1;

		if (config["descriptor"])
		{
			YAML::Node descriptorConfig = config["descriptor"];

			DescriptorParams *params = new DescriptorParams();
			params->patchSize = descriptorConfig["patchSize"].as<double>();
			params->bandNumber = descriptorConfig["bandNumber"].as<int>();
			params->bandWidth = descriptorConfig["bandWidth"].as<double>();
			params->bidirectional = descriptorConfig["bidirectional"].as<bool>();
			params->useProjection = descriptorConfig["useProjection"].as<bool>();
			params->sequenceBin = descriptorConfig["sequenceBin"].as<double>();
			params->sequenceStat = ExecutionParams::getStatType(descriptorConfig["sequenceStat"].as<std::string>());

			getInstance()->descriptorParams = params;
		}

		if (config["clustering"])
		{
			YAML::Node clusteringConfig = config["clustering"];

			ClusteringParams *params = new ClusteringParams();
			params->implementation = ExecutionParams::getClusteringImplementation(clusteringConfig["implementation"].as<std::string>());
			params->metric = ExecutionParams::getMetricType(clusteringConfig["metric"].as<std::string>());
			params->clusterNumber = clusteringConfig["clusterNumber"].as<int>();
			params->maxIterations = clusteringConfig["maxIterations"].as<int>();
			params->stopThreshold = clusteringConfig["stopThreshold"].as<double>();
			params->attempts = clusteringConfig["attempts"].as<int>();
			params->cacheLocation = clusteringConfig["cacheLocation"].as<std::string>();
			params->generateElbowCurve = clusteringConfig["generateElbowCurve"].as<bool>();
			params->generateDistanceMatrix = clusteringConfig["generateDistanceMatrix"].as<bool>();

			getInstance()->clusteringParams = params;
		}

		if (config["cloudSmoothing"])
		{
			YAML::Node smoothingConfig = config["cloudSmoothing"];

			CloudSmoothingParams *params = new CloudSmoothingParams();
			params->useSmoothing = smoothingConfig["useSmoothing"].as<bool>();
			params->radius = smoothingConfig["radius"].as<double>();
			params->sigma = smoothingConfig["sigma"].as<double>();

			getInstance()->cloudSmoothingParams = params;
		}

		if (config["syntheticCloud"])
		{
			YAML::Node synthCloudConfig = config["syntheticCloud"];

			SyntheticCloudsParams *params = new SyntheticCloudsParams();
			params->useSynthetic = synthCloudConfig["generateCloud"].as<bool>();
			params->synCloudType = ExecutionParams::getSynCloudType(synthCloudConfig["type"].as<std::string>());

			getInstance()->syntheticCloudParams = params;
		}

		if (config["metricTesting"])
		{
			YAML::Node metricTestConfig = config["metricTesting"];

			MetricTestingParams *params = new MetricTestingParams();
			params->metric = ExecutionParams::getMetricType(metricTestConfig["metric"].as<std::string>());

			std::vector<std::string> args;
			std::string str = metricTestConfig["args"].as<std::string>();
			std::istringstream iss(str.substr(0, str.find_last_of('#')));
			std::copy(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(), std::back_inserter(args));
			params->args = args;

			getInstance()->metricTestingParams = params;
		}
	}
	catch (std::exception &_ex)
	{
		std::cout << "ERROR: " << _ex.what() << std::endl;
		loadOk = false;
	}

	return loadOk;
}
