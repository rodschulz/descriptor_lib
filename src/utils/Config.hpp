/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <string>
#include "ExecutionParams.hpp"

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

private:
	// Constructor
	Config();

	// Instance storing the current execution params loaded from the config file
	ExecutionParams params;
};
