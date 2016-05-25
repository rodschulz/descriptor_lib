/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <string>
#include "ExecutionParams.hpp"

#define OUTPUT_FOLDER		"./output/"
#define CLOUD_FILE_EXTENSION	".pcd"
#define DEBUG_PREFIX		"DEBUG_"

class Config
{
public:
	// Destructor
	~Config();

	// Returns the instance of the singleton
	static Config *getInstance()
	{
		static Config instance = Config();
		return &instance;
	}

	// Loads the configuration file
	static bool load(const std::string &_filename);

	// Returns the current execution params according to the loaded configuration file
	static inline ExecutionParams getExecutionParams()
	{
		return getInstance()->params;
	}

	// Returns a boolean value indicating if the debug generation is enabled
	static bool debugEnabled()
	{
		return getInstance()->debug;
	}

private:
	// Constructor
	Config();

	// Instance storing the current execution params loaded from the config file
	ExecutionParams params;

	// Flag indicating if the debug generation is enabled or not
	bool debug;
};
