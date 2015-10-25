/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <string>
#include <stdlib.h>
#include <boost/algorithm/string.hpp>
#include <boost/functional/hash.hpp>
#include <boost/lexical_cast.hpp>
#include "Helper.h"
#include "../factories/MetricFactory.h"

enum SynCloudType
{
	CLOUD_NONE, CLOUD_CUBE, CLOUD_CYLINDER, CLOUD_SPHERE
};

enum SmoothingType
{
	SMOOTHING_NONE, SMOOTHING_GAUSSIAN, SMOOTHING_MLS
};

enum SequenceStat
{
	STAT_NONE, STAT_MEAN, STAT_MEDIAN
};

enum ClusteringImplementation
{
	CLUSTERING_NONE, CLUSTERING_OPENCV, CLUSTERING_CUSTOM, CLUSTERING_STOCHASTIC
};

class ExecutionParams
{
public:
	ExecutionParams();
	~ExecutionParams()
	{
	}

	static SynCloudType getSynCloudType(const std::string &_type);
	static SmoothingType getSmoothingType(const std::string &_type);
	static SequenceStat getStatType(const std::string &_type);
	static ClusteringImplementation getClusteringImplementation(const std::string &_type);
	static MetricType getMetricType(const std::string &_type);

	std::string getHash() const;
	double getBandsAngularRange() const;
	double getBandsAngularStep() const;
	int getSequenceLength() const;

	bool normalExecution;				// Flag indicating if the execution has to be normal or a clustering evaluation

	std::string inputLocation;			// Location of the input file
	int targetPoint;				// Target point

	double patchSize;				// Search radius used with the SEARCH_RADIUS method
	double normalEstimationRadius;			// Radius used to calculate normals
	int bandNumber;					// Number of bands to sample
	double bandWidth;				// Width of each band
	bool bidirectional;				// Flag indicating if each band has to be analyzed bidirectional or not
	bool useProjection;				// Flag indicating if angle calculation must be done by projecting the angles over the band's plane

	double sequenceBin;				// Size of the bins used in the sequence construction
	SequenceStat sequenceStat;			// Statistic to use in the sequence calculation

	bool useSynthetic;				// Flag indicating if a synthetic has to be used
	SynCloudType synCloudType;			// Desired synthetic cloud

	SmoothingType smoothingType;			// Type of smoothing algorithm to use
	double gaussianSigma;				// Sigma used for the gaussian smoothning
	double gaussianRadius;				// Search radius used for the gaussian smoothing
	double mlsRadius;				// Search radius used for the mls smoothing

	bool genElbowCurve;				// Flag indicating if an elbow graph has to be generated
	ClusteringImplementation implementation;	// Implementation of clustering to be used
	MetricType metric;				// Type of metric to use in clustering execution
	int clusters;					// Number of clusters used in the clustering test
	int maxIterations;				// Clustering max iterations
	double stopThreshold;				// Clustering stop threshold
	int attempts;					// Number of attemtps to try when clustering
	std::string cacheLocation;			// Location of the cachefiles
};
