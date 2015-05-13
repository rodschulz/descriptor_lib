/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <string>

using namespace std;

struct ExecutionParams
{
	string inputLocation;	// Location of the input file
	int targetPoint;	// Target point

	double searchRadius;	// Search radius used with the SEARCH_RADIUS method
	int bandNumber;		// Number of bands to sample
	double bandWidth;	// Width of each band
	bool bidirectional;	// Flag indicating if each band has to be analyzed bidirectional or not
	bool radialBands;	// Flag indicating if the bands are radial or longitudinal

	ExecutionParams()
	{
		inputLocation = "";
		searchRadius = 0.05;
		targetPoint = 1000;
		bandNumber = 4;
		bandWidth = 0.01;
		bidirectional = true;
		radialBands = false;
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
};

