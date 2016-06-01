/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <string>
#include <vector>
#include <math.h>
#include <boost/shared_ptr.hpp>

// Forward declaration of metric shared pointer
class Metric;
typedef boost::shared_ptr<Metric> MetricPtr;

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Set of enumerations defining some easy-to-read values for some parameters
/////////////////////////////////////////////////////////////////////////////////////////////////////////
enum SequenceStat
{
	STAT_MEAN, STAT_MEDIAN
};

enum ClusteringImplementation
{
	CLUSTERING_OPENCV, CLUSTERING_CUSTOM, CLUSTERING_STOCHASTIC
};

enum MetricType
{
	METRIC_EUCLIDEAN, METRIC_CLOSEST_PERMUTATION, METRIC_CLOSEST_PERMUTATION_WITH_CONFIDENCE
};

enum SynCloudType
{
	CLOUD_CUBE, CLOUD_CYLINDER, CLOUD_SPHERE, CLOUD_HALF_SPHERE, CLOUD_PLANE
};
/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Set of structures groupping functionality-related parameters
/////////////////////////////////////////////////////////////////////////////////////////////////////////
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
		patchSize = 1;
		bandNumber = 4;
		bandWidth = 0.05;
		bidirectional = false;
		useProjection = false;
		sequenceBin = 0.05;
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
};

/**
 * Structure grouping the params involved in the custering process
 */
struct ClusteringParams
{
	ClusteringImplementation implementation; // Implementation of clustering to be used
	MetricPtr metric; // Metric to use in clustering execution
	int clusterNumber; // Number of clusters used in the clustering test
	int maxIterations; // Clustering max iterations
	double stopThreshold; // Clustering stop threshold
	int attempts; // Number of attemtps to try when clustering
	bool generateElbowCurve; // Flag indicating if an elbow graph has to be generated
	bool generateDistanceMatrix; // Flag indicating if the distance matrix image has to be generated

	ClusteringParams()
	{
		implementation = CLUSTERING_OPENCV;
		metric = MetricPtr();
		clusterNumber = 5;
		maxIterations = 10000;
		stopThreshold = 0.1;
		attempts = 1;
		generateElbowCurve = false;
		generateDistanceMatrix = false;
	}
};

/**
 * Structure grouping the params for the cloud smoothing routine
 */
struct CloudSmoothingParams
{
	bool useSmoothing; // Flag indicating if the smoothing has to be performed or not
	double sigma; // Sigma used for the gaussian smoothning
	double radius; // Search radius used for the gaussian smoothing

	CloudSmoothingParams()
	{
		useSmoothing = false;
		sigma = 4;
		radius = 0.005;
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
};
/////////////////////////////////////////////////////////////////////////////////////////////////////////
