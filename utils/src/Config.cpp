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
#include <yaml-cpp/node/impl.h>
#include "MetricFactory.hpp"
#include "Utils.hpp"

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
		getInstance()->config = config;

		getInstance()->debug = config["debug"].as<bool>(false);
		getInstance()->targetPoint = config["targetPoint"].as<int>(-1);
		getInstance()->normalEstimationRadius = config["normalEstimationRadius"].as<double>(-1);
		getInstance()->cacheLocation = config["cacheLocation"].as<std::string>("");

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
			params->sequenceStat = Utils::getStatType(descriptorConfig["sequenceStat"].as<std::string>());

			getInstance()->descriptorParams = params;
		}

		if (config["clustering"])
		{
			YAML::Node clusteringConfig = config["clustering"];

			ClusteringParams *params = new ClusteringParams();
			params->implementation = Utils::getClusteringImplementation(clusteringConfig["implementation"].as<std::string>());
			std::vector<std::string> metricDetails = clusteringConfig["metric"].as<std::vector<std::string> >();
			params->metric = MetricFactory::createMetric(Utils::getMetricType(metricDetails[0]), metricDetails);
			params->clusterNumber = clusteringConfig["clusterNumber"].as<int>();
			params->maxIterations = clusteringConfig["maxIterations"].as<int>();
			params->stopThreshold = clusteringConfig["stopThreshold"].as<double>();
			params->attempts = clusteringConfig["attempts"].as<int>();
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
			params->synCloudType = Utils::getSynCloudType(synthCloudConfig["type"].as<std::string>());

			getInstance()->syntheticCloudParams = params;
		}

		if (config["metricTesting"])
		{
			YAML::Node metricTestConfig = config["metricTesting"];

			MetricTestingParams *params = new MetricTestingParams();
			std::vector<std::string> metricDetails = metricTestConfig["metric"].as<std::vector<std::string> >();
			params->metric = MetricFactory::createMetric(Utils::getMetricType(metricDetails[0]), metricDetails);

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
