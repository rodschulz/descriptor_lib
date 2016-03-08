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
	~Config();

	// Returns the instance of the singleton
	static Config *getInstance()
	{
		static Config instance = Config();
		return &instance;
	}

	// Loads the configuration file
	static bool load(const std::string &_filename);
	static inline ExecutionParams getExecutionParams()
	{
		return getInstance()->params;
	}

private:
	Config();
	ExecutionParams params;
};