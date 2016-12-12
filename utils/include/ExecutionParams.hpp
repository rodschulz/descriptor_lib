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
	BOOST_STRINGIZE(STAT_MEAN)
};


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


////////////////////////////////////////////////////////////////////////////////
// Set of structures grouping functionality-related parameters
////////////////////////////////////////////////////////////////////////////////
/**
 * Structure grouping the parameters involved in the descriptor's calculation
 */
struct DescriptorParams
{
	double patchSize; // Search radius for the KNN search method
	int bandNumber; // Number of bands to use in the descriptor
	double bandWidth; // Width of each descriptor's band
	bool bidirectional; // Indicates if each band has to be analyzed bidirectionally or not
	bool useProjection; // Indicates if the normal's angle calculation has to be projecting the angles over the band's plane or just use the raw angle
	double sequenceBin; // Size of the bins used in the sequence construction
	SequenceStat sequenceStat; // Statistic to use in the sequence calculation

	DescriptorParams()
	{
		patchSize = 0.05;
		bandNumber = 4;
		bandWidth = 0.01;
		bidirectional = true;
		useProjection = true;
		sequenceBin = 0.01;
		sequenceStat = STAT_MEAN;
	}

	// Returns sequence's length on each bands (number of bins) according to the current config
	int getSequenceLength() const
	{
		return (bidirectional ? patchSize * 2.0 : patchSize) / sequenceBin;
	}

	// Returns the angular range of the bands according to the current config
	double getBandsAngularRange() const
	{
		if (bidirectional)
			return M_PI;
		else
			return 2 * M_PI;
	}

	// Returns the angular step of the bands according to the current config
	double getBandsAngularStep() const
	{
		return getBandsAngularRange() / bandNumber;
	}

	// Returns a string holding the structure's information
	std::string toString() const
	{
		std::stringstream stream;
		stream << std::boolalpha
			   << "patchSize:" << patchSize
			   << " bandNumber:" << bandNumber
			   << " bandWidth:" << bandWidth
			   << " bidirectional:" << bidirectional
			   << " useProjection:" << useProjection
			   << " sequenceBin:" << sequenceBin
			   << " sequenceStat:" << seqStat[sequenceStat];
		return stream.str();
	}
};

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
	bool generateElbowCurve; // Flag indicating if an elbow graph has to be generated
	bool generateDistanceMatrix; // Flag indicating if the distance matrix image has to be generated

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

	// Returns a string holding the struct's information
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

	CloudSmoothingParams()
	{
		useSmoothing = false;
		sigma = 2;
		radius = 0.02;
	}

	// Returns a string holding the struct's information
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
 * Structure defing the params for the generation of synthetic clouds
 */
struct SyntheticCloudsParams
{
	bool useSynthetic; // Flag indicating if a synthetic cloud has to be generated
	SynCloudType synCloudType; // Desired synthetic cloud

	SyntheticCloudsParams()
	{
		useSynthetic = false;
		synCloudType = CLOUD_SPHERE;
	}

	// Returns a string holding the struct's information
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

	MetricTestingParams()
	{
		metric = MetricPtr();
	}

	// Returns a string holding the struct's information
	std::string toString() const
	{
		std::stringstream stream;
		stream << std::boolalpha << " metric:[" << metricType[metric->getType()] << "," << boost::algorithm::join(metric->getConstructionParams(), ",") << "]";
		return stream.str();
	}
};
/////////////////////////////////////////////////////////////////////////////////////////////////////////
