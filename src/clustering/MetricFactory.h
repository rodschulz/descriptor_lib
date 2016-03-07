/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <stdarg.h>
#include "EuclideanMetric.h"
#include "ClosestPermutationMetric.h"
#include "../utils/ExecutionParams.h"

class MetricFactory
{
public:
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

private:
	MetricFactory();
	~MetricFactory();
};

