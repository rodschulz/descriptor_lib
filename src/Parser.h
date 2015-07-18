/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include "ExecutionParams.h"

class Parser
{
public:
	static void printUsage();
	static ExecutionParams parseExecutionParams(int _argn, char **_argv);
private:
	Parser();
	~Parser();
};
