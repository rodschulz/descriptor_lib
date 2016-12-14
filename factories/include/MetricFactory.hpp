/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <stdarg.h>
#include <vector>
#include <string>
#include <boost/algorithm/string.hpp>
#include "EuclideanMetric.hpp"
#include "ClosestPermutationMetric.hpp"
#include "ClosestPermutationWithConfidenceMetric.hpp"
#include "Config.hpp"

class MetricFactory
{
public:
	// Creates a metric instance according to the given parameters
	static MetricPtr createMetric(const MetricType &type_, const std::vector<std::string> _args = std::vector<std::string>())
	{
		switch (type_)
		{
		case METRIC_EUCLIDEAN:
			return MetricPtr(new EuclideanMetric());

		case METRIC_CLOSEST_PERMUTATION:
		case METRIC_CLOSEST_PERMUTATION_WITH_CONFIDENCE:
		{
			int size = 0;

			// Check if should use the configuration to "calculate" the params or read them
			if (boost::iequals("conf", _args[1]))
				size = Config::getDescriptorParams().getSequenceLength();
			else
				size = atoi(_args[1].c_str());

			if (type_ == METRIC_CLOSEST_PERMUTATION)
				return MetricPtr(new ClosestPermutationMetric(size));
			else
				return MetricPtr(new ClosestPermutationWithConfidenceMetric(size));
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

