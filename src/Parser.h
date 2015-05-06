/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <string>

using namespace std;

typedef enum PatchGenerationMethod
{
	SEARCH_RADIUS, K_NEIGHBORS
} PatchGenerationMethod;

struct Params
{
	string inputLocation;		// Location of the input file
	int targetPoint;		// Target point

	PatchGenerationMethod method;	// Method used to generate the surface patch to use
	double searchRadius;		// Search radius used with the SEARCH_RADIUS method
	int neighborsNumber;		// Number of neighbors to sample, used with the method K_NEIGHBORS

	int bandNumber;			// Number of bands to sample
	double bandWidth;		// Width of each band
	bool bidirectional;		// Flag indicating if each band has to be analyzed bidirectional or not

	Params()
	{
		inputLocation = "";
		method = SEARCH_RADIUS;
		searchRadius = 0.05;
		neighborsNumber = 100;
		targetPoint = 1000;
		bandNumber = 4;
		bandWidth = 0.01;
		bidirectional = true;
	}
};

class Parser
{
public:
	static void printUsage();
	static Params parseExecutionParams(int _argn, char **_argv);
private:
	Parser();
	~Parser();
};

