/**
 * Author: rodrigo
 * 2015
 */
#include "ExecutionParams.h"
#include "Utils.hpp"

ExecutionParams::ExecutionParams()
{
	executionType = EXECUTION_NONE;

	inputLocation = "";
	targetPoint = 1000;

	patchSize = 0.05;
	normalEstimationRadius = -1;
	bandNumber = 4;
	bandWidth = 0.01;
	bidirectional = true;
	useProjection = true;

	sequenceBin = 0.01;
	sequenceStat = STAT_NONE;

	useSynthetic = false;
	synCloudType = CLOUD_NONE;

	smoothingType = SMOOTHING_NONE;
	gaussianSigma = 2;
	gaussianRadius = 0.02;
	mlsRadius = 0.02;

	genElbowCurve = false;
	genDistanceMatrix = false;

	labelData = false;
	centersLocation = "";

	implementation = CLUSTERING_NONE;
	metric = METRIC_NONE;
	clusters = 5;
	maxIterations = 10000;
	stopThreshold = 0.001;
	attempts = 1;
	cacheLocation = "";
	useConfidence = false;

	targetMetric = METRIC_NONE;
	metricArgs = std::vector<std::string>();
}

ExecutionType ExecutionParams::getExecutionType(const std::string &_type)
{
	if (boost::iequals(_type, "descriptor"))
		return EXECUTION_DESCRIPTOR;
	else if (boost::iequals(_type, "clustering"))
		return EXECUTION_CLUSTERING;
	else if (boost::iequals(_type, "metric"))
		return EXECUTION_METRIC;
	return EXECUTION_NONE;
}

SynCloudType ExecutionParams::getSynCloudType(const std::string &_type)
{
	if (boost::iequals(_type, "cube"))
		return CLOUD_CUBE;
	else if (boost::iequals(_type, "cylinder"))
		return CLOUD_CYLINDER;
	else if (boost::iequals(_type, "sphere"))
		return CLOUD_SPHERE;
	return CLOUD_NONE;
}

SmoothingType ExecutionParams::getSmoothingType(const std::string &_type)
{
	if (boost::iequals(_type, "gaussian"))
		return SMOOTHING_GAUSSIAN;
	else if (boost::iequals(_type, "mls"))
		return SMOOTHING_MLS;
	return SMOOTHING_NONE;
}

SequenceStat ExecutionParams::getStatType(const std::string &_type)
{
	if (boost::iequals(_type, "mean"))
		return STAT_MEAN;
	else if (boost::iequals(_type, "median"))
		return STAT_MEDIAN;
	return STAT_NONE;
}

ClusteringImplementation ExecutionParams::getClusteringImplementation(const std::string &_type)
{
	if (boost::iequals(_type, "opencv"))
		return CLUSTERING_OPENCV;
	else if (boost::iequals(_type, "custom"))
		return CLUSTERING_CUSTOM;
	else if (boost::iequals(_type, "stochastic"))
		return CLUSTERING_STOCHASTIC;
	return CLUSTERING_NONE;
}

MetricType ExecutionParams::getMetricType(const std::string &_type)
{
	if (boost::iequals(_type, "euclidean"))
		return METRIC_EUCLIDEAN;
	else if (boost::iequals(_type, "closest"))
		return METRIC_CLOSEST_PERMUTATION;
	return METRIC_NONE;
}

std::string ExecutionParams::getHash() const
{
	std::string str = "";

	std::string inputFile = inputLocation;
	std::string sdasd = inputLocation.substr(0, 2);
	if (inputLocation.substr(0, 2).compare("./") == 0)
		inputFile = inputLocation.substr(2);

	str += "input=" + inputFile;
	str += "-patchSize=" + boost::lexical_cast<std::string>(patchSize);
	str += "-normalEstimationRadius=" + boost::lexical_cast<std::string>(normalEstimationRadius);
	str += "-bandNumber=" + boost::lexical_cast<std::string>(bandNumber);
	str += "-bandWidth=" + boost::lexical_cast<std::string>(bandWidth);
	str += "-bidirectional=" + boost::lexical_cast<std::string>(bidirectional);
	str += "-useProjection=" + boost::lexical_cast<std::string>(useProjection);
	str += "-sequenceBin=" + boost::lexical_cast<std::string>(sequenceBin);
	str += "-sequenceStat=" + boost::lexical_cast<std::string>(sequenceStat);
	str += "-smoothingType=" + boost::lexical_cast<std::string>(smoothingType);
	if (smoothingType == SMOOTHING_GAUSSIAN)
	{
		str += "-gaussianSigma=" + boost::lexical_cast<std::string>(gaussianSigma);
		str += "-gaussianRadius=" + boost::lexical_cast<std::string>(gaussianRadius);
	}
	else if (smoothingType == SMOOTHING_MLS)
		str += "-mlsRadius=" + boost::lexical_cast<std::string>(mlsRadius);

	boost::hash<std::string> strHash;
	return Utils::num2Hex(strHash(str));
}

double ExecutionParams::getBandsAngularRange() const
{
	if (bidirectional)
		return M_PI;
	else
		return 2 * M_PI;
}

double ExecutionParams::getBandsAngularStep() const
{
	return getBandsAngularRange() / bandNumber;
}

int ExecutionParams::getSequenceLength() const
{
	return (bidirectional ? patchSize * 2.0 : patchSize) / sequenceBin;
}
