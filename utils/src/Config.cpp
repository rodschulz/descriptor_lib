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

Config::Config()
{
	debug = false;
	descriptorParams = NULL;
	clusteringParams = NULL;
	smoothingParams = NULL;
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

		if (config["descriptor"])
		{
			YAML::Node descriptorConfig = config["descriptor"];

			DescriptorParams *params = new DescriptorParams();
			params->patchSize = descriptorConfig["patchSize"].as<double>();
			params->normalRadius = descriptorConfig["normalRadius"].as<double>();
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
		}

		if (config["cloudSmoothing"])
		{
		}

		if (config["syntheticCloud"])
		{
		}

		// Execution type
//		getInstance()->params.executionType = ExecutionParams::getExecutionType(config["executionType"].as<std::string>());
//
//		YAML::Node desc = config["descriptor"];
//		YAML::Node tg = desc["targetPoint"];
//
//		// Descriptor calculation
//		getInstance()->params.targetPoint = config["descriptor"]["targetPoint"].as<int>();
//
//		getInstance()->params.patchSize = config["descriptor"]["patchSize"].as<double>();
//		getInstance()->params.normalEstimationRadius = config["descriptor"]["normalRadius"].as<double>();
//		getInstance()->params.bandNumber = config["descriptor"]["bandNumber"].as<int>();
//		getInstance()->params.bandWidth = config["descriptor"]["bandWidth"].as<double>();
//		getInstance()->params.bidirectional = config["descriptor"]["bidirectional"].as<bool>();
//		getInstance()->params.useProjection = config["descriptor"]["useProjection"].as<bool>();
//
//		getInstance()->params.sequenceBin = config["descriptor"]["sequenceBin"].as<double>();
//		getInstance()->params.sequenceStat = ExecutionParams::getStatType(config["descriptor"]["sequenceStat"].as<std::string>());
//
//		// Clustering generation
//		getInstance()->params.genElbowCurve = config["clustering"]["generateElbowCurve"].as<bool>();
//		getInstance()->params.genDistanceMatrix = config["clustering"]["generateDistanceMatrix"].as<bool>();
//
//		getInstance()->params.labelData = config["clustering"]["labelData"].as<bool>();
//		getInstance()->params.centersLocation = config["clustering"]["centersLocation"].as<std::string>();
//
//		getInstance()->params.implementation = ExecutionParams::getClusteringImplementation(config["clustering"]["implementation"].as<std::string>());
//
//		getInstance()->params.metric = ExecutionParams::getMetricType(config["clustering"]["metric"].as<std::string>());
//		getInstance()->params.clusters = config["clustering"]["clusters"].as<int>();
//		getInstance()->params.maxIterations = config["clustering"]["maxIterations"].as<int>();
//		getInstance()->params.stopThreshold = config["clustering"]["stopThreshold"].as<double>();
//		getInstance()->params.attempts = config["clustering"]["attempts"].as<int>();
//		getInstance()->params.cacheLocation = config["clustering"]["cacheLocation"].as<std::string>();
//		getInstance()->params.useConfidence = config["clustering"]["useConfidence"].as<bool>();
//
//		// Metric testcase evaluation
//		getInstance()->params.targetMetric = ExecutionParams::getMetricType(config["metric"]["targetMetric"].as<std::string>());
//
//		std::vector<std::string> args;
//		std::string str = config["metric"]["metricArgs"].as<std::string>();
//		std::istringstream iss(str.substr(0, str.find_last_of('#')));
//		std::copy(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(), std::back_inserter(args));
//		getInstance()->params.metricArgs = args;
//
//		// Cloud smoothing params
//		getInstance()->params.smoothingType = ExecutionParams::getSmoothingType(config["cloudSmoothing"]["type"].as<std::string>());
//
//		getInstance()->params.gaussianSigma = config["cloudSmoothing"]["gaussian"]["sigma"].as<double>();
//		getInstance()->params.gaussianRadius = config["cloudSmoothing"]["gaussian"]["radius"].as<double>();
//
//		getInstance()->params.mlsRadius = config["cloudSmoothing"]["mls"]["radius"].as<double>();
//
//		// Synthetic cloud generation
//		getInstance()->params.useSynthetic = config["syntheticClouds"]["generateCloud"].as<bool>();
//		getInstance()->params.synCloudType = ExecutionParams::getSynCloudType(config["syntheticClouds"]["type"].as<std::string>());
	}
	catch (std::exception &_ex)
	{
		std::cout << "ERROR: " << _ex.what() << std::endl;
		loadOk = false;
	}

	return loadOk;
}
