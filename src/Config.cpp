/**
 * Author: rodrigo
 * 2015
 */
#include "Config.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <iterator>
#include <math.h>
#include <vector>
#include <boost/algorithm/string.hpp>

Config::Config()
{
}

Config::~Config()
{
}

bool Config::load(const string &_filename, int _argn, char **_argv)
{
	bool loadOk = true;

	string line;
	ifstream inputFile;
	inputFile.open(_filename.c_str(), fstream::in);
	if (inputFile.is_open())
	{
		while (getline(inputFile, line))
		{
			if (line.empty() || line[0] == '#')
				continue;

			vector<string> tokens;
			istringstream iss(line);
			copy(istream_iterator<string>(iss), istream_iterator<string>(), back_inserter(tokens));

			parse(tokens[0], tokens[1]);
		}
		inputFile.close();

		if (_argn < 2 && !getInstance()->params.useSynthetic)
		{
			loadOk = false;
			cout << "Not enough parameters\n";
		}
		else if (_argn >= 2)
			getInstance()->params.inputLocation = _argv[1];

	}
	else
	{
		loadOk = false;
		cout << "Unable to open input: " << _filename;
	}

	return loadOk;
}

void Config::parse(const string _key, const string _value)
{
	// Descriptor calculation
	if (boost::iequals(_key, "targetPoint"))
		getInstance()->params.targetPoint = atoi(_value.c_str());

	else if (boost::iequals(_key, "patchSize"))
		getInstance()->params.patchSize = atof(_value.c_str());

	else if (boost::iequals(_key, "normalRadius"))
		getInstance()->params.normalEstimationRadius = atof(_value.c_str());

	else if (boost::iequals(_key, "bandNumber"))
		getInstance()->params.bandNumber = atoi(_value.c_str());

	else if (boost::iequals(_key, "bandWidth"))
		getInstance()->params.bandWidth = atof(_value.c_str());

	else if (boost::iequals(_key, "sequenceBin"))
		getInstance()->params.sequenceBin = atof(_value.c_str());

	else if (boost::iequals(_key, "bidirectional"))
		getInstance()->params.bidirectional = boost::iequals(_value, "true");

	else if (boost::iequals(_key, "radialBands"))
		getInstance()->params.radialBands = boost::iequals(_value, "true");

	// Use of synthetic clouds
	else if (boost::iequals(_key, "useSynthetic"))
		getInstance()->params.useSynthetic = boost::iequals(_value, "true");

	else if (boost::iequals(_key, "synCloudType"))
		getInstance()->params.synCloudType = ExecutionParams::getType(_value);
}
