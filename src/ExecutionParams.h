/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <string>
#include <stdlib.h>

using namespace std;

enum SynCloudType
{
	NONE, CUBE, CYLINDER, SPHERE,
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
		synCloudType = NONE;

		smoothCloud = false;
	}

	~ExecutionParams()
	{
	}

	static SynCloudType getType(const string &_type)
	{
		int type = atoi(_type.c_str());

		if (0 < type || type < 3)
			return (SynCloudType) type;
		else
			return NONE;
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

	bool smoothCloud;		// Flag indicating if smoothing has to be done
};
