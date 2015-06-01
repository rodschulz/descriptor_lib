/**
 * Author: rodrigo
 * 2015
 */
#include "Parser.h"
#include <stdlib.h>
#include <iostream>
#include <cstring>
#include "Helper.h"

Parser::Parser()
{
}

Parser::~Parser()
{
}

void Parser::printUsage()
{
	cout << "Usage:\n";
	cout << "   TestDescriptores <input_file or synthetic cloud> <target_point_index> [OPTIONS]\n\n";
	cout << "   -sc INT\tRuns the application using a synthetic cloud. Options are 1=cube, 2=cylinder and 3=sphere\n";
	cout << "   -r FLOAT\tSets the search radius used to extract the surface patch\n";
	cout << "   -n FLOAT\tSets the radius used to perform the normal estimation. If none is given, then the search radius is used\n";
	cout << "   -b INT\tSets the number of bands to create for analysis (by default 4)\n";
	cout << "   -w FLOAT\tSets the band's width (by default 0.05)\n";
	cout << "   -u\t\tWhen present each band is defined from the target point to the \n\t\tpatch's border and not across the point\n";
	cout << "   -R\t\tWhen present bands are defined as radial zones and the width of each\n\t\twill be defined by the patch's size the number of bands\n\n";
}

ExecutionParams Parser::parseExecutionParams(int _argn, char **_argv)
{
	ExecutionParams params;

	if (_argn > 3)
	{
		if (!Helper::isNumber(_argv[2]))
		{
			cout << "ERROR: wrong execution params. Target point invalid (" << _argv[2] << ")\n";
		}
		else
		{
			int i;
			if (strcmp(_argv[1], "-sc") == 0)
			{
				// Using a synthetic cloud
				params.useSynthetic = true;
				params.synCloudType = getType(_argv[2]);
				params.targetPoint = atoi(_argv[3]);
				i = 4;
			}
			else
			{
				// Not using a synthetic cloud
				params.inputLocation = _argv[1];
				params.targetPoint = atoi(_argv[2]);
				i = 3;
			}

			bool normalRadiusSet = false;
			while (i < _argn)
			{
				if (strcmp(_argv[i], "-R") == 0)
				{
					// Set radial flag
					params.radialBands = true;
				}
				else if (strcmp(_argv[i], "-r") == 0)
				{
					// Set search radius
					params.searchRadius = atof(_argv[++i]);
					if (!normalRadiusSet)
						params.normalEstimationRadius = params.searchRadius;
				}
				else if (strcmp(_argv[i], "-n") == 0)
				{
					// Set the normal estimation radius
					params.normalEstimationRadius = atof(_argv[++i]);
					normalRadiusSet = true;
				}
				else if (strcmp(_argv[i], "-b") == 0)
				{
					// Set band number
					params.bandNumber = atoi(_argv[++i]);
				}
				else if (strcmp(_argv[i], "-w") == 0)
				{
					// Set band width
					params.bandWidth = atof(_argv[++i]);
				}
				else if (strcmp(_argv[i], "-u") == 0)
				{
					// Set as unidirectional band analysis
					params.bidirectional = false;
				}

				i++;
			}
		}
	}

	return params;
}

SynCloudType Parser::getType(const string &_type)
{
	int type = atoi(_type.c_str());

	if (0 < type || type < 3)
		return (SynCloudType) type;
	else
		return NONE;
}
