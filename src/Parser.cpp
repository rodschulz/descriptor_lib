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
	cout << "Usage:\n" << "   -m INT\tSets the patch extraction method, 0 for radius method, \n\t\t1 for k neighbors method (by default 0)\n" << "   -r FLOAT\tSets the search radius used to extract the surface patch, \n\t\tused with method 0 (by default 0.01)\n" << "   -n INT\tSets the number of neighbors desired in the surface patch, \n\t\tused with method 1 (by default 1000)\n" << "   -b INT\tSets the number of bands to create for analysis (by default 4)\n" << "   -w FLOAT\tSets the band's width (by default 0.05)\n" << "   -u\t\tWhen present each band is defined from the target point to the \n\t\tpatch's border and not across the point\n\n";
}

ExecutionParams Parser::parseExecutionParams(int _argn, char **_argv)
{
	ExecutionParams params;

	if (_argn > 3)
	{
		if (!Helper::isNumber(_argv[2]))
		{
			cout << "ERROR: wrong execution param. Target point invalid (" << _argv[2] << ")\n";
		}
		else
		{

			params.inputLocation = _argv[1];
			params.targetPoint = atoi(_argv[2]);

			int i = 3;
			while (i < _argn)
			{
				if (strcmp(_argv[i], "-m") == 0)
				{
					// Set method
					params.method = (PatchGenerationMethod) atoi(_argv[++i]);
				}
				else if (strcmp(_argv[i], "-r") == 0)
				{
					// Set search radius
					params.searchRadius = atof(_argv[++i]);
				}
				else if (strcmp(_argv[i], "-n") == 0)
				{
					// Set desired neighbors number
					params.neighborsNumber = atoi(_argv[++i]);
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
