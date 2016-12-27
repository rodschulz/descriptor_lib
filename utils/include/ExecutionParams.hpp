/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <string>
#include <vector>
#include <math.h>
#include <sstream>
#include <boost/shared_ptr.hpp>
#include <boost/algorithm/string/join.hpp>
#include <boost/algorithm/string.hpp>
#include <plog/Log.h>
#include "Metric.hpp"


namespace Params
{
////////////////////////////////////////////////////////////////////////////////
// Set of enumerations defining some easy-to-read values for some parameters
////////////////////////////////////////////////////////////////////////////////
enum Statistic
{
	STAT_MEAN,
	STAT_MEDIAN,
	STAT_HISTOGRAM_10,
	STAT_HISTOGRAM_20,
	STAT_HISTOGRAM_30,
	STAT_HISTOGRAM_BIN_10,
	STAT_HISTOGRAM_BIN_20,
	STAT_HISTOGRAM_BIN_30,
};
static std::string stat[] =
{
	BOOST_STRINGIZE(STAT_MEAN),
	BOOST_STRINGIZE(STAT_MEDIAN),
	BOOST_STRINGIZE(STAT_HISTOGRAM_10),
	BOOST_STRINGIZE(STAT_HISTOGRAM_20),
	BOOST_STRINGIZE(STAT_HISTOGRAM_30),
	BOOST_STRINGIZE(STAT_HISTOGRAM_BIN_10),
	BOOST_STRINGIZE(STAT_HISTOGRAM_BIN_20),
	BOOST_STRINGIZE(STAT_HISTOGRAM_BIN_30),
};

static inline Statistic toStatType(const std::string &type_)
{
	if (boost::iequals(type_, "mean"))
		return STAT_MEAN;
	else if (boost::iequals(type_, "median"))
		return STAT_MEDIAN;
	else if (boost::iequals(type_, "hist10"))
		return STAT_HISTOGRAM_10;
	else if (boost::iequals(type_, "hist20"))
		return STAT_HISTOGRAM_20;
	else if (boost::iequals(type_, "hist30"))
		return STAT_HISTOGRAM_30;
	else if (boost::iequals(type_, "hb10"))
		return STAT_HISTOGRAM_BIN_10;
	else if (boost::iequals(type_, "hb20"))
		return STAT_HISTOGRAM_BIN_20;
	else if (boost::iequals(type_, "hb30"))
		return STAT_HISTOGRAM_BIN_30;

	LOGW << "Wrong stat type, assuming MEAN";
	return STAT_MEAN;
}

static inline std::string toString(const Statistic &stat_)
{
	switch (stat_)
	{
	default:
		LOGW << "Wrong stat type, assuming MEAN";
	case STAT_MEAN:
		return "mean";
	case STAT_MEDIAN:
		return "median";
	case STAT_HISTOGRAM_10:
		return "hist10";
	case STAT_HISTOGRAM_20:
		return "hist20";
	case STAT_HISTOGRAM_30:
		return "hist30";
	case STAT_HISTOGRAM_BIN_10:
		return "hb10";
	case STAT_HISTOGRAM_BIN_20:
		return "hb20";
	case STAT_HISTOGRAM_BIN_30:
		return "hb30";
	}
}


/**************************************************/
/**************************************************/
enum ClusteringImplementation
{
	CLUSTERING_OPENCV,
	CLUSTERING_KMEANS,
	CLUSTERING_STOCHASTIC,
};
static std::string clusteringImp[] =
{
	BOOST_STRINGIZE(CLUSTERING_OPENCV),
	BOOST_STRINGIZE(CLUSTERING_KMEANS),
	BOOST_STRINGIZE(CLUSTERING_STOCHASTIC),
};

static inline ClusteringImplementation toClusteringImp(const std::string &type_)
{
	if (boost::iequals(type_, "opencv"))
		return CLUSTERING_OPENCV;
	else if (boost::iequals(type_, "kmeans"))
		return CLUSTERING_KMEANS;
	else if (boost::iequals(type_, "stochastic"))
		return CLUSTERING_STOCHASTIC;

	LOGW << "Wrong clustering implementation, assuming OPENCV";
	return CLUSTERING_OPENCV;
}


/**************************************************/
/**************************************************/
enum SynCloudType
{
	CLOUD_CUBE,
	CLOUD_CYLINDER,
	CLOUD_SPHERE,
	CLOUD_HALF_SPHERE,
	CLOUD_PLANE
};
static std::string cloudType[] = {
	BOOST_STRINGIZE(CLOUD_CUBE),
	BOOST_STRINGIZE(CLOUD_CYLINDER),
	BOOST_STRINGIZE(CLOUD_SPHERE),
	BOOST_STRINGIZE(CLOUD_HALF_SPHERE),
	BOOST_STRINGIZE(CLOUD_PLANE)
};

static inline SynCloudType toSynCloudType(const std::string &type_)
{
	if (boost::iequals(type_, "cube"))
		return CLOUD_CUBE;
	else if (boost::iequals(type_, "cylinder"))
		return CLOUD_CYLINDER;
	else if (boost::iequals(type_, "sphere"))
		return CLOUD_SPHERE;
	else if (boost::iequals(type_, "half_sphere"))
		return CLOUD_HALF_SPHERE;
	else if (boost::iequals(type_, "plane"))
		return CLOUD_PLANE;

	LOGW << "Wrong synthetic cloud type, assuming SPHERE";
	return CLOUD_SPHERE;
}
}


/**************************************************/
/**************************************************/
/**
 * Structure grouping the params involved in the clustering process
 */
struct ClusteringParams
{
	Params::ClusteringImplementation implementation; // Implementation of clustering to be used
	MetricPtr metric; // Metric to use in clustering execution
	int clusterNumber; // Number of clusters used in the clustering test
	int maxIterations; // Clustering max iterations
	double stopThreshold; // Clustering stop threshold
	int attempts; // Number of attempts to try when clustering
	bool generateElbowCurve; // True if an elbow graph has to be generated
	bool generateDistanceMatrix; // True if the distance matrix image has to be generated

	/**************************************************/
	ClusteringParams()
	{
		implementation = Params::CLUSTERING_OPENCV;
		metric = MetricPtr();
		clusterNumber = 5;
		maxIterations = 10000;
		stopThreshold = 0.001;
		attempts = 1;
		generateElbowCurve = false;
		generateDistanceMatrix = false;
	}

	/**************************************************/
	std::string toString() const
	{
		std::ostringstream stream;
		stream << std::boolalpha
			   << "implementation:" << Params::clusteringImp[implementation]
			   << " metric:[" << metricType[metric->getType()] << "," << boost::algorithm::join(metric->getConstructionParams(), ",") << "]"
			   << " clusterNumber:" << clusterNumber
			   << " maxIterations:" << maxIterations
			   << " stopThreshold:" << stopThreshold
			   << " attempts:" << attempts
			   << " generateElbowCurve:" << generateElbowCurve
			   << " generateDistanceMatrix:" << generateDistanceMatrix;

		return stream.str();
	}
};

/**
 * Structure grouping the params for the cloud smoothing routine
 */
struct CloudSmoothingParams
{
	bool useSmoothing; // Flag indicating if the smoothing has to be performed or not
	double sigma; // Sigma used for the gaussian smoothing
	double radius; // Search radius used for the gaussian smoothing

	/**************************************************/
	CloudSmoothingParams()
	{
		useSmoothing = false;
		sigma = 2;
		radius = 0.02;
	}

	/**************************************************/
	std::string toString() const
	{
		std::stringstream stream;
		stream << std::boolalpha
			   << "useSmoothing:" << useSmoothing
			   << " sigma:" << sigma
			   << " radius:" << radius;
		return stream.str();
	}
};

/**
 * Structure defining the params for the generation of synthetic clouds
 */
struct SyntheticCloudsParams
{
	bool useSynthetic; // Flag indicating if a synthetic cloud has to be generated
	Params::SynCloudType synCloudType; // Desired synthetic cloud

	/**************************************************/
	SyntheticCloudsParams()
	{
		useSynthetic = false;
		synCloudType = Params::CLOUD_SPHERE;
	}

	/**************************************************/
	std::string toString() const
	{
		std::stringstream stream;
		stream << std::boolalpha
			   << "useSynthetic:" << useSynthetic
			   << " synCloudType:" << Params::cloudType[synCloudType];
		return stream.str();
	}
};
