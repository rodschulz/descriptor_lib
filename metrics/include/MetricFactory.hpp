/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <stdarg.h>
#include <vector>
#include <string>
#include <boost/algorithm/string.hpp>
#include "ClosestPermutationMetric.hpp"
#include "EuclideanMetric.hpp"
#include "Config.hpp"

class MetricFactory
{
public:
	// Creates a metric instance according to the given parameters
	static MetricPtr createMetric(const MetricType &_type, std::vector<std::string> _args)
	{
		switch (_type)
		{
			case METRIC_EUCLIDEAN:
				return MetricPtr(new EuclideanMetric());

			case METRIC_CLOSEST_PERMUTATION:
			{
				int size = 0;

				// Check if should use the configuration to "calculate" the params or read them
				if (boost::iequals("conf", _args[1]))
					size = Config::getDescriptorParams().getSequenceLength();
				else
					size = atoi(_args[1].c_str());

				return MetricPtr(new ClosestPermutationMetric(size, false));
			}

			default:
				return MetricPtr();
		}
	}

private:
	// Constructor
	MetricFactory();
	// Destructor
	~MetricFactory();
};

