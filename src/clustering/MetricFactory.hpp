/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <stdarg.h>
#include <vector>
#include <string>
#include "../utils/ExecutionParams.hpp"
#include "ClosestPermutationMetric.hpp"
#include "EuclideanMetric.hpp"

class MetricFactory
{
public:
	// Creates a metric instance according to the given parameters
	static MetricPtr createMetric(const MetricType &_type, ...)
	{
		MetricPtr metric;
		switch (_type)
		{
			case METRIC_EUCLIDEAN:
				metric = MetricPtr(new EuclideanMetric());
				break;

			case METRIC_CLOSEST_PERMUTATION:
			{
				va_list argsList;
				va_start(argsList, _type);
				metric = MetricPtr(new ClosestPermutationMetric(va_arg(argsList, int), va_arg(argsList, int) == 1));
			}
				break;

			default:
				metric = MetricPtr();
		}

		return metric;
	}

	// Creates a metric instance according to the given parameters
	static MetricPtr createMetric(const MetricType &_type, const std::vector<std::string> &_args)
	{
		MetricPtr metric;
		switch (_type)
		{
			case METRIC_EUCLIDEAN:
				metric = MetricPtr(new EuclideanMetric());
				break;

			case METRIC_CLOSEST_PERMUTATION:
				metric = MetricPtr(new ClosestPermutationMetric(atoi(_args[0].c_str()), _args[1].compare("true") == 0));
				break;

			default:
				metric = MetricPtr();
		}

		return metric;
	}

private:
	MetricFactory();
	~MetricFactory();
};

