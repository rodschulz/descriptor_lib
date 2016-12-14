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
#include "Metric.hpp"


////////////////////////////////////////////////////////////////////////////////
// Set of enumerations defining some easy-to-read values for some parameters
////////////////////////////////////////////////////////////////////////////////
enum SequenceStat
{
	STAT_MEAN,
	STAT_MEDIAN
};
static std::string seqStat[] =
{
	BOOST_STRINGIZE(STAT_MEAN),
	BOOST_STRINGIZE(STAT_MEDIAN)
};

SequenceStat toStatType(const std::string &type_)
{
	if (boost::iequals(type_, "mean"))
		return STAT_MEAN;
	else if (boost::iequals(type_, "median"))
		return STAT_MEDIAN;

	std::cout << "WARNING: wrong stat type, assuming MEAN";
	return STAT_MEAN;
}


/**************************************************/
/**************************************************/
enum ClusteringImplementation
{
	CLUSTERING_OPENCV,
	CLUSTERING_KMEANS,
	CLUSTERING_STOCHASTIC,
	CLUSTERING_KMEDOIDS
};
static std::string clusteringImp[] =
{
	BOOST_STRINGIZE(CLUSTERING_OPENCV),
	BOOST_STRINGIZE(CLUSTERING_KMEANS),
	BOOST_STRINGIZE(CLUSTERING_STOCHASTIC),
	BOOST_STRINGIZE(CLUSTERING_KMEDOIDS)
};

ClusteringImplementation toClusteringImp(const std::string &type_)
{
	if (boost::iequals(type_, "opencv"))
		return CLUSTERING_OPENCV;
	else if (boost::iequals(type_, "kmeans"))
		return CLUSTERING_KMEANS;
	else if (boost::iequals(type_, "stochastic"))
		return CLUSTERING_STOCHASTIC;
	else if (boost::iequals(type_, "kmedoids"))
		return CLUSTERING_KMEDOIDS;

	std::cout << "WARNING: wrong clustering implementation, assuming OPENCV";
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

SynCloudType toSynCloudType(const std::string &type_)
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

	std::cout << "WARNING: wrong synthetic cloud type, assuming SPHERE";
	return CLOUD_SPHERE;
}



/**************************************************/
/**************************************************/


// enum DescriptorType
// {
// 	DESCRIPTOR_DCH,
// 	DESCRIPTOR_SHOT,
// 	DESCRIPTOR_USC,
// 	DESCRIPTOR_PFH,
// 	DESCRIPTOR_ROPS,
// };
// static std::string descType[] = {
// 	BOOST_STRINGIZE(DESCRIPTOR_DCH),
// 	BOOST_STRINGIZE(DESCRIPTOR_SHOT),
// 	BOOST_STRINGIZE(DESCRIPTOR_USC),
// 	BOOST_STRINGIZE(DESCRIPTOR_PFH),
// 	BOOST_STRINGIZE(DESCRIPTOR_ROPS)
// };


////////////////////////////////////////////////////////////////////////////////
// Set of structures grouping functionality-related parameters
////////////////////////////////////////////////////////////////////////////////
/**
 * Structure grouping the parameters involved in the descriptor's calculation
 */
// struct DescriptorParams
// {
// 	DescriptorType type; // Descriptor type
// 	double searchRadius; // Search radius for the KNN search method
// 	int bandNumber; // Number of bands of the descriptor
// 	double bandWidth; // Width of each band
// 	bool bidirectional; // True if each band is bidirectional
// 	bool useProjection; // True if the angle calculation is using a projection
// 	double sequenceBin; // Size of the bins used in the sequence construction
// 	SequenceStat sequenceStat; // Statistic used in the descriptor

// 	/**************************************************/
// 	DescriptorParams()
// 	{
// 		type = DESCRIPTOR_DCH;
// 		searchRadius = 0.05;
// 		bandNumber = 4;
// 		bandWidth = 0.01;
// 		bidirectional = true;
// 		useProjection = true;
// 		sequenceBin = 0.01;
// 		sequenceStat = STAT_MEAN;
// 	}

// 	/**************************************************/
// 	int getSequenceLength() const
// 	{
// 		return (bidirectional ? searchRadius * 2.0 : searchRadius) / sequenceBin;
// 	}

// 	/**************************************************/
// 	double getBandsAngularRange() const
// 	{
// 		if (bidirectional)
// 			return M_PI;
// 		else
// 			return 2 * M_PI;
// 	}

// 	/**************************************************/
// 	double getBandsAngularStep() const
// 	{
// 		return getBandsAngularRange() / bandNumber;
// 	}

// 	/**************************************************/
// 	std::string toString() const
// 	{
// 		std::stringstream stream;
// 		stream << std::boolalpha
// 			   << "type:" << descType[type]
// 			   << " searchRadius:" << searchRadius
// 			   << " bandNumber:" << bandNumber
// 			   << " bandWidth:" << bandWidth
// 			   << " bidirectional:" << bidirectional
// 			   << " useProjection:" << useProjection
// 			   << " sequenceBin:" << sequenceBin
// 			   << " sequenceStat:" << seqStat[sequenceStat];
// 		return stream.str();
// 	}
// };

/**
 * Structure grouping the params involved in the clustering process
 */
struct ClusteringParams
{
	ClusteringImplementation implementation; // Implementation of clustering to be used
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
		implementation = CLUSTERING_OPENCV;
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
			   << "implementation:" << clusteringImp[implementation]
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
	SynCloudType synCloudType; // Desired synthetic cloud

	/**************************************************/
	SyntheticCloudsParams()
	{
		useSynthetic = false;
		synCloudType = CLOUD_SPHERE;
	}

	/**************************************************/
	std::string toString() const
	{
		std::stringstream stream;
		stream << std::boolalpha
			   << "useSynthetic:" << useSynthetic
			   << " synCloudType:" << cloudType[synCloudType];
		return stream.str();
	}
};

/**
 * Structure grouping the params for the metric testing routines
 */
struct MetricTestingParams
{
	MetricPtr metric; // Type of metric to be tested

	/**************************************************/
	MetricTestingParams()
	{
		metric = MetricPtr();
	}

	/**************************************************/
	std::string toString() const
	{
		std::stringstream stream;
		stream << std::boolalpha << "metric:[" << metricType[metric->getType()] << "," << boost::algorithm::join(metric->getConstructionParams(), ",") << "]";
		return stream.str();
	}
};
/////////////////////////////////////////////////////////////////////////////////////////////////////////
