/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <string>
#include <stdlib.h>
#include <boost/algorithm/string.hpp>

using namespace std;

enum SynCloudType
{
	CLOUD_NONE, CLOUD_CUBE, CLOUD_CYLINDER, CLOUD_SPHERE,
};

enum SmoothingType
{
	SMOOTHING_NONE, SMOOTHING_GAUSSIAN, SMOOTHING_MLS,
};

class ExecutionParams
{
public:
	ExecutionParams()
	{
		inputLocation = "";
		patchSize = 0.05;
		normalEstimationRadius = -1;
		targetPoint = 1000;
		bandNumber = 4;
		bandWidth = 0.01;
		sequenceBin = 0.01;
		bidirectional = true;
		radialBands = false;

		useSynthetic = false;
		synCloudType = CLOUD_NONE;

		smoothingType = SMOOTHING_NONE;
		gaussianSigma = 2;
		gaussianRadius = 0.02;
		mlsRadius = 0.02;
	}

	~ExecutionParams()
	{
	}

	static SynCloudType getSynCloudType(const string &_type)
	{
		if (boost::iequals(_type, "cube"))
			return CLOUD_CUBE;
		else if (boost::iequals(_type, "cylinder"))
			return CLOUD_CYLINDER;
		else if (boost::iequals(_type, "sphere"))
			return CLOUD_SPHERE;
		return CLOUD_NONE;
	}

	static SmoothingType getSmoothingType(const string &_type)
	{
		if (boost::iequals(_type, "gaussian"))
			return SMOOTHING_GAUSSIAN;
		else if (boost::iequals(_type, "mls"))
			return SMOOTHING_MLS;
		return SMOOTHING_NONE;
	}

	string inputLocation;		// Location of the input file
	int targetPoint;		// Target point

	double patchSize;		// Search radius used with the SEARCH_RADIUS method
	double normalEstimationRadius;	// Radius used to calculate normals
	int bandNumber;			// Number of bands to sample
	double bandWidth;		// Width of each band
	double sequenceBin;		// Size of the bins used in the sequence construction
	bool bidirectional;		// Flag indicating if each band has to be analyzed bidirectional or not
	bool radialBands;		// Flag indicating if the bands are radial or longitudinal

	bool useSynthetic;		// Flag indicating if a synthetic has to be used
	SynCloudType synCloudType;	// Desired synthetic cloud

	SmoothingType smoothingType;	// Type of smoothing algorithm to use
	double gaussianSigma;		// Sigma used for the gaussian smoothning
	double gaussianRadius;		// Search radius used for the gaussian smoothing
	double mlsRadius;		// Search radius used for the mls smoothing
};
