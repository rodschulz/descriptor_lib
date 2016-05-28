/**
 * Author: rodrigo
 * 2015
 */
#include "ExecutionParams.hpp"
#include <boost/algorithm/string.hpp>
#include <boost/functional/hash.hpp>
#include <boost/lexical_cast.hpp>
#include "Utils.hpp"

ExecutionParams::ExecutionParams()
{
//	executionType = EXECUTION_NONE;

	inputLocation = "";
	targetPoint = 1000;

	patchSize = 0.05;
	normalEstimationRadius = -1;
	bandNumber = 4;
	bandWidth = 0.01;
	bidirectional = true;
	useProjection = true;

	sequenceBin = 0.01;
	sequenceStat = STAT_MEAN;

	useSynthetic = false;
	synCloudType = CLOUD_HALF_SPHERE;

//	smoothingType = SMOOTHING_NONE;
	gaussianSigma = 2;
	gaussianRadius = 0.02;
	mlsRadius = 0.02;

	genElbowCurve = false;
	genDistanceMatrix = false;

	labelData = false;
	centersLocation = "";

	implementation = CLUSTERING_OPENCV;
	metric = METRIC_EUCLIDEAN;
	clusters = 5;
	maxIterations = 10000;
	stopThreshold = 0.001;
	attempts = 1;
	cacheLocation = "";
	useConfidence = false;

	targetMetric = METRIC_EUCLIDEAN;
	metricArgs = std::vector<std::string>();
}

//ExecutionType ExecutionParams::getExecutionType(const std::string &_type)
//{
//	if (boost::iequals(_type, "descriptor"))
//		return EXECUTION_DESCRIPTOR;
//	else if (boost::iequals(_type, "clustering"))
//		return EXECUTION_CLUSTERING;
//	else if (boost::iequals(_type, "metric"))
//		return EXECUTION_METRIC;
//	return EXECUTION_NONE;
//}

SynCloudType ExecutionParams::getSynCloudType(const std::string &_type)
{
	if (boost::iequals(_type, "cube"))
		return CLOUD_CUBE;
	else if (boost::iequals(_type, "cylinder"))
		return CLOUD_CYLINDER;
	else if (boost::iequals(_type, "sphere"))
		return CLOUD_SPHERE;
	else if (boost::iequals(_type, "half_sphere"))
		return CLOUD_HALF_SPHERE;
	else if (boost::iequals(_type, "plane"))
		return CLOUD_PLANE;
	{
		std::cout << "WARNING: wrong synthetic cloud type, assuming SPHERE";
		return CLOUD_SPHERE;
	}
}

SequenceStat ExecutionParams::getStatType(const std::string &_type)
{
	if (boost::iequals(_type, "mean"))
		return STAT_MEAN;
	else if (boost::iequals(_type, "median"))
		return STAT_MEDIAN;
	{
		std::cout << "WARNING: wrong stat type, assuming MEAN";
		return STAT_MEAN;
	}
}

ClusteringImplementation ExecutionParams::getClusteringImplementation(const std::string &_type)
{
	if (boost::iequals(_type, "opencv"))
		return CLUSTERING_OPENCV;
	else if (boost::iequals(_type, "custom"))
		return CLUSTERING_CUSTOM;
	else if (boost::iequals(_type, "stochastic"))
		return CLUSTERING_STOCHASTIC;
	else
	{
		std::cout << "WARNING: wrong clustering implementation, assuming OPENCV";
		return CLUSTERING_OPENCV;
	}
}

MetricType ExecutionParams::getMetricType(const std::string &_type)
{
	if (boost::iequals(_type, "euclidean"))
		return METRIC_EUCLIDEAN;
	else if (boost::iequals(_type, "closest"))
		return METRIC_CLOSEST_PERMUTATION;
	else if (boost::iequals(_type, "closest_with_confidence"))
		return METRIC_CLOSEST_PERMUTATION_WITH_CONFIDENCE;
	{
		std::cout << "WARNING: wrong metric type, assuming EUCLIDEAN";
		return METRIC_EUCLIDEAN;
	}
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
	str += "-normalRadius=" + boost::lexical_cast<std::string>(normalEstimationRadius);
	str += "-bandNumber=" + boost::lexical_cast<std::string>(bandNumber);
	str += "-bandWidth=" + boost::lexical_cast<std::string>(bandWidth);
	str += "-bidirectional=" + boost::lexical_cast<std::string>(bidirectional);
	str += "-useProjection=" + boost::lexical_cast<std::string>(useProjection);
	str += "-sequenceBin=" + boost::lexical_cast<std::string>(sequenceBin);
	str += "-sequenceStat=" + boost::lexical_cast<std::string>(sequenceStat);
//	str += "-useSmoothing=" + boost::lexical_cast<std::string>(useSmoothing); //TODO fix this
//	str += "-smoothingSigma=" + boost::lexical_cast<std::string>(sigma);
//	str += "-smoothingRadius=" + boost::lexical_cast<std::string>(radius);

	boost::hash<std::string> strHash;
	return Utils::num2Hex(strHash(str));
}

//double ExecutionParams::getBandsAngularRange() const
//{
//	if (bidirectional)
//		return M_PI;
//	else
//		return 2 * M_PI;
//}

//double ExecutionParams::getBandsAngularStep() const
//{
//	return getBandsAngularRange() / bandNumber;
//}

//int ExecutionParams::getSequenceLength() const
//{
//	return (bidirectional ? patchSize * 2.0 : patchSize) / sequenceBin;
//}
