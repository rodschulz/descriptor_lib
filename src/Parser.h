/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <string>

using namespace std;

enum SynCloudType
{
	NONE, CUBE, CYLINDER, SPHERE,
};

struct ExecutionParams
{
	string inputLocation;		// Location of the input file
	int targetPoint;		// Target point

	double searchRadius;		// Search radius used with the SEARCH_RADIUS method
	double normalEstimationRadius;	// Radius used to calculate normals
	int bandNumber;			// Number of bands to sample
	double bandWidth;		// Width of each band
	bool bidirectional;		// Flag indicating if each band has to be analyzed bidirectional or not
	bool radialBands;		// Flag indicating if the bands are radial or longitudinal

	bool useSynthetic;		// Flag indicating if a synthetic has to be used
	SynCloudType synCloudType;	// Desired synthetic cloud

	ExecutionParams()
	{
		inputLocation = "";
		searchRadius = 0.05;
		normalEstimationRadius = -1;
		targetPoint = 1000;
		bandNumber = 4;
		bandWidth = 0.01;
		bidirectional = true;
		radialBands = false;

		useSynthetic = false;
		synCloudType = NONE;
	}
};

class Parser
{
public:
	static void printUsage();
	static ExecutionParams parseExecutionParams(int _argn, char **_argv);
private:
	Parser();
	~Parser();

	static SynCloudType getType(const string &_type);
};

