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
	cout << "Usage:\n"
		<< "   TestDescriptores <input_file> <target_point_index> [OPTIONS]\n\n"
		<< "   -r FLOAT\tSets the search radius used to extract the surface patch, \n"
		<< "   -b INT\tSets the number of bands to create for analysis (by default 4)\n"
		<< "   -w FLOAT\tSets the band's width (by default 0.05)\n"
		<< "   -u\t\tWhen present each band is defined from the target point to the \n\t\tpatch's border and not across the point\n"
		<< "   -R\t\tWhen present bands are defined as radial zones and the width of each\n\t\twill be defined by the patch's size the number of bands\n\n";
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
				if (strcmp(_argv[i], "-R") == 0)
				{
					// Set radial flag
					params.radialBands = true;
				}
				else if (strcmp(_argv[i], "-r") == 0)
				{
					// Set search radius
					params.searchRadius = atof(_argv[++i]);
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
