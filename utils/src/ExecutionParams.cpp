/**
 * Author: rodrigo
 * 2015
 */
#include "ExecutionParams.hpp"
#include <boost/algorithm/string.hpp>
#include <boost/functional/hash.hpp>
#include <boost/lexical_cast.hpp>
#include "Utils.hpp"

//ExecutionParams::ExecutionParams()
//{
////	executionType = EXECUTION_NONE;
//
//	inputLocation = "";
//	targetPoint = 1000;
//
//	patchSize = 0.05;
//	normalEstimationRadius = -1;
//	bandNumber = 4;
//	bandWidth = 0.01;
//	bidirectional = true;
//	useProjection = true;
//
//	sequenceBin = 0.01;
//	sequenceStat = STAT_MEAN;
//
//	useSynthetic = false;
//	synCloudType = CLOUD_HALF_SPHERE;
//
////	smoothingType = SMOOTHING_NONE;
//	gaussianSigma = 2;
//	gaussianRadius = 0.02;
//	mlsRadius = 0.02;
//
//	genElbowCurve = false;
//	genDistanceMatrix = false;
//
//	labelData = false;
//	centersLocation = "";
//
//	implementation = CLUSTERING_OPENCV;
//	metric = METRIC_EUCLIDEAN;
//	clusters = 5;
//	maxIterations = 10000;
//	stopThreshold = 0.001;
//	attempts = 1;
//	cacheLocation = "";
//	useConfidence = false;
//
//	targetMetric = METRIC_EUCLIDEAN;
//	metricArgs = std::vector<std::string>();
//}

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

//SynCloudType ExecutionParams::getSynCloudType(const std::string &_type)
//{
//	if (boost::iequals(_type, "cube"))
//		return CLOUD_CUBE;
//	else if (boost::iequals(_type, "cylinder"))
//		return CLOUD_CYLINDER;
//	else if (boost::iequals(_type, "sphere"))
//		return CLOUD_SPHERE;
//	else if (boost::iequals(_type, "half_sphere"))
//		return CLOUD_HALF_SPHERE;
//	else if (boost::iequals(_type, "plane"))
//		return CLOUD_PLANE;
//	{
//		std::cout << "WARNING: wrong synthetic cloud type, assuming SPHERE";
//		return CLOUD_SPHERE;
//	}
//}
//
//SequenceStat ExecutionParams::getStatType(const std::string &_type)
//{
//	if (boost::iequals(_type, "mean"))
//		return STAT_MEAN;
//	else if (boost::iequals(_type, "median"))
//		return STAT_MEDIAN;
//	{
//		std::cout << "WARNING: wrong stat type, assuming MEAN";
//		return STAT_MEAN;
//	}
//}
//
//ClusteringImplementation ExecutionParams::getClusteringImplementation(const std::string &_type)
//{
//	if (boost::iequals(_type, "opencv"))
//		return CLUSTERING_OPENCV;
//	else if (boost::iequals(_type, "custom"))
//		return CLUSTERING_CUSTOM;
//	else if (boost::iequals(_type, "stochastic"))
//		return CLUSTERING_STOCHASTIC;
//	else
//	{
//		std::cout << "WARNING: wrong clustering implementation, assuming OPENCV";
//		return CLUSTERING_OPENCV;
//	}
//}
//
//MetricType ExecutionParams::getMetricType(const std::string &_type)
//{
//	if (boost::iequals(_type, "euclidean"))
//		return METRIC_EUCLIDEAN;
//	else if (boost::iequals(_type, "closest"))
//		return METRIC_CLOSEST_PERMUTATION;
//	else if (boost::iequals(_type, "closest_with_confidence"))
//		return METRIC_CLOSEST_PERMUTATION_WITH_CONFIDENCE;
//	{
//		std::cout << "WARNING: wrong metric type, assuming EUCLIDEAN";
//		return METRIC_EUCLIDEAN;
//	}
//}

//std::string ExecutionParams::getHash(const std::string _inputFile, const double _normalEstimationRadius, const DescriptorParams &_descriptorParams, const CloudSmoothingParams &_smoothingParams) const
//{
//	// TODO improve this using the file's hash instead of just the file's name
//	std::string filename = inputLocation;
//	if (_inputFile.substr(0, 2).compare("./") == 0)
//		filename = inputLocation.substr(2);
//
//	std::string str = "";
//	str += "input=" + filename;
//	str += "-normalEstimationRadius=" + boost::lexical_cast<std::string>(_normalEstimationRadius);
//	str += "-patchSize=" + boost::lexical_cast<std::string>(_descriptorParams.patchSize);
//	str += "-bandNumber=" + boost::lexical_cast<std::string>(_descriptorParams.bandNumber);
//	str += "-bandWidth=" + boost::lexical_cast<std::string>(_descriptorParams.bandWidth);
//	str += "-bidirectional=" + boost::lexical_cast<std::string>(_descriptorParams.bidirectional);
//	str += "-useProjection=" + boost::lexical_cast<std::string>(_descriptorParams.useProjection);
//	str += "-sequenceBin=" + boost::lexical_cast<std::string>(_descriptorParams.sequenceBin);
//	str += "-sequenceStat=" + boost::lexical_cast<std::string>(_descriptorParams.sequenceStat);
//	if (_smoothingParams.useSmoothing)
//	{
//		str += "-useSmoothing=" + boost::lexical_cast<std::string>(_smoothingParams.useSmoothing);
//		str += "-smoothingSigma=" + boost::lexical_cast<std::string>(_smoothingParams.sigma);
//		str += "-smoothingRadius=" + boost::lexical_cast<std::string>(_smoothingParams.radius);
//	}
//
//	boost::hash<std::string> strHash;
//	return Utils::num2Hex(strHash(str));
//}

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
