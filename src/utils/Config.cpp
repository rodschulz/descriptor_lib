/**
 * Author: rodrigo
 * 2015
 */
#include "Config.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <iterator>
#include <math.h>
#include <vector>
#include <boost/algorithm/string.hpp>

Config::Config()
{
}

Config::~Config()
{
}

bool Config::load(const std::string &_filename, int _argn, char **_argv)
{
	bool loadOk = true;

	std::string line;
	std::ifstream inputFile;
	inputFile.open(_filename.c_str(), std::fstream::in);
	if (inputFile.is_open())
	{
		while (getline(inputFile, line))
		{
			if (line.empty() || line[0] == '#')
				continue;

			std::vector<std::string> tokens;
			std::istringstream iss(line);
			std::copy(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(), std::back_inserter(tokens));

			parse(tokens[0], tokens[1]);
		}
		inputFile.close();

		if (_argn < 2 && !getInstance()->params.useSynthetic)
		{
			loadOk = false;
			std::cout << "Not enough parameters\n";
		}
		else if (_argn >= 2)
			getInstance()->params.inputLocation = _argv[1];

	}
	else
	{
		loadOk = false;
		std::cout << "Unable to open input: " << _filename;
	}

	return loadOk;
}

void Config::parse(const std::string _key, const std::string _value)
{
	/*****************************/
	// Execution type selection
	if (boost::iequals(_key, "normalExec"))
		getInstance()->params.normalExecution = boost::iequals(_value, "true");

	/*****************************/
	// Clustering parameters
	else if (boost::iequals(_key, "genElbowCurve"))
		getInstance()->params.genElbowCurve = boost::iequals(_value, "true");

	else if (boost::iequals(_key, "implementation"))
		getInstance()->params.implementation = ExecutionParams::getClusteringImplementation(_value);

	else if (boost::iequals(_key, "metric"))
			getInstance()->params.metric = ExecutionParams::getMetricType(_value);

	else if (boost::iequals(_key, "clusters"))
		getInstance()->params.clusters = atoi(_value.c_str());

	else if (boost::iequals(_key, "maxIter"))
		getInstance()->params.maxIterations = atoi(_value.c_str());

	else if (boost::iequals(_key, "stopThres"))
		getInstance()->params.stopThreshold = atof(_value.c_str());

	else if (boost::iequals(_key, "attempts"))
		getInstance()->params.attempts = atoi(_value.c_str());

	else if (boost::iequals(_key, "cacheLocation"))
		getInstance()->params.cacheLocation = _value.c_str();

	/*****************************/
	// Descriptor calculation
	else if (boost::iequals(_key, "targetPoint"))
		getInstance()->params.targetPoint = atoi(_value.c_str());

	else if (boost::iequals(_key, "patchSize"))
		getInstance()->params.patchSize = atof(_value.c_str());

	else if (boost::iequals(_key, "normalRadius"))
		getInstance()->params.normalEstimationRadius = atof(_value.c_str());

	else if (boost::iequals(_key, "bandNumber"))
		getInstance()->params.bandNumber = atoi(_value.c_str());

	else if (boost::iequals(_key, "bandWidth"))
		getInstance()->params.bandWidth = atof(_value.c_str());

	else if (boost::iequals(_key, "bidirectional"))
		getInstance()->params.bidirectional = boost::iequals(_value, "true");

	else if (boost::iequals(_key, "useProjection"))
		getInstance()->params.useProjection = boost::iequals(_value, "true");

	/*****************************/
	// Sequence calculation
	else if (boost::iequals(_key, "sequenceBin"))
		getInstance()->params.sequenceBin = atof(_value.c_str());

	else if (boost::iequals(_key, "sequenceStat"))
		getInstance()->params.sequenceStat = ExecutionParams::getStatType(_value);

	/*****************************/
	// Use of synthetic clouds
	else if (boost::iequals(_key, "useSynthetic"))
		getInstance()->params.useSynthetic = boost::iequals(_value, "true");

	else if (boost::iequals(_key, "synCloudType"))
		getInstance()->params.synCloudType = ExecutionParams::getSynCloudType(_value);

	/*****************************/
	// Smoothing params
	else if (boost::iequals(_key, "smoothingType"))
		getInstance()->params.smoothingType = ExecutionParams::getSmoothingType(_value);

	else if (boost::iequals(_key, "gaussianSigma"))
		getInstance()->params.gaussianSigma = atof(_value.c_str());

	else if (boost::iequals(_key, "gaussianRadius"))
		getInstance()->params.gaussianRadius = atof(_value.c_str());

	else if (boost::iequals(_key, "mlsRadius"))
		getInstance()->params.mlsRadius = atof(_value.c_str());
}
