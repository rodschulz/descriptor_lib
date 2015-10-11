/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <string>
#include "ExecutionParams.h"

class Config
{
public:
	~Config();

	// Returns the instance of the singleton
	static Config *getInstance()
	{
		static Config *instance = new Config();
		return instance;
	}

	// Loads the configuration file
	static bool load(const std::string &_filename, int _argn, char **_argv);

	static inline ExecutionParams getExecutionParams()
	{
		return getInstance()->params;
	}

private:
	Config();
	static void parse(const std::string _key, const std::string _value);

	ExecutionParams params;
};
