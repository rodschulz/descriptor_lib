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

enum SynCloudType
{
	CLOUD_NONE, CLOUD_CUBE, CLOUD_CYLINDER, CLOUD_SPHERE,
};

enum SmoothingType
{
	SMOOTHING_NONE, SMOOTHING_GAUSSIAN, SMOOTHING_MLS,
};

enum SequenceStat
{
	STAT_NONE, STAT_MEAN, STAT_MEDIAN
};

enum ClusteringImplementation
{
	CLUSTERING_NONE, CLUSTERING_OPENCV, CLUSTERING_CUSTOM
};

class ExecutionParams
{
public:
	ExecutionParams()
	{
		normalExecution = true;

		targetPoint = 1000;

		patchSize = 0.05;
		normalEstimationRadius = -1;
		bandNumber = 4;
		bandWidth = 0.01;
		bidirectional = true;
		radialBands = false;
		useProjection = true;

		sequenceBin = 0.01;
		sequenceStat = STAT_NONE;

		useSynthetic = false;
		synCloudType = CLOUD_NONE;

		smoothingType = SMOOTHING_NONE;
		gaussianSigma = 2;
		gaussianRadius = 0.02;
		mlsRadius = 0.02;

		clusters = 5;
		maxIterations = 10000;
		stopThreshold = 0.001;
		showElbow = false;
		implementation = CLUSTERING_NONE;
	}

	~ExecutionParams()
	{
	}

	static SynCloudType getSynCloudType(const std::string &_type)
	{
		if (boost::iequals(_type, "cube"))
			return CLOUD_CUBE;
		else if (boost::iequals(_type, "cylinder"))
			return CLOUD_CYLINDER;
		else if (boost::iequals(_type, "sphere"))
			return CLOUD_SPHERE;
		return CLOUD_NONE;
	}

	static SmoothingType getSmoothingType(const std::string &_type)
	{
		if (boost::iequals(_type, "gaussian"))
			return SMOOTHING_GAUSSIAN;
		else if (boost::iequals(_type, "mls"))
			return SMOOTHING_MLS;
		return SMOOTHING_NONE;
	}

	static SequenceStat getStatType(const std::string &_type)
	{
		if (boost::iequals(_type, "mean"))
			return STAT_MEAN;
		else if (boost::iequals(_type, "median"))
			return STAT_MEDIAN;
		return STAT_NONE;
	}

	static ClusteringImplementation getClusteringImplementation(const std::string &_type)
	{
		if (boost::iequals(_type, "opencv"))
			return CLUSTERING_OPENCV;
		else if (boost::iequals(_type, "custom"))
			return CLUSTERING_CUSTOM;
		return CLUSTERING_NONE;
	}

	std::string getHash() const
	{
		std::string str = "";
		str += "input=" + inputLocation;
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
		return Helper::toHexString(strHash(str));
	}

	bool normalExecution;				// Flag indicating if the execution has to be normal or a clustering evaluation

	std::string inputLocation;			// Location of the input file
	int targetPoint;				// Target point

	double patchSize;				// Search radius used with the SEARCH_RADIUS method
	double normalEstimationRadius;			// Radius used to calculate normals
	int bandNumber;					// Number of bands to sample
	double bandWidth;				// Width of each band
	bool bidirectional;				// Flag indicating if each band has to be analyzed bidirectional or not
	bool radialBands;				// Flag indicating if the bands are radial or longitudinal
	bool useProjection;				// Flag indicating if angle calculation must be done by projecting the angles over the band's plane

	double sequenceBin;				// Size of the bins used in the sequence construction
	SequenceStat sequenceStat;			// Statistic to use in the sequence calculation

	bool useSynthetic;				// Flag indicating if a synthetic has to be used
	SynCloudType synCloudType;			// Desired synthetic cloud

	SmoothingType smoothingType;			// Type of smoothing algorithm to use
	double gaussianSigma;				// Sigma used for the gaussian smoothning
	double gaussianRadius;				// Search radius used for the gaussian smoothing
	double mlsRadius;				// Search radius used for the mls smoothing

	bool showElbow;					// Flag indicating if an elbow graph has to be done
	ClusteringImplementation implementation;	// Implementation of clustering to be used
	int clusters;					// Number of clusters used in the clustering test
	int maxIterations;				// Clustering max iterations
	double stopThreshold;				// Clustering stop threshold
	std::string cacheLocation;			// Location of the cachefiles
};
