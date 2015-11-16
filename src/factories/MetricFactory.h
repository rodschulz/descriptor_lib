/**
 * Author: rodrigo
 * 2015
 */
#pragma once
#include "../clustering/EuclideanMetric.h"
#include "../clustering/ClosestPermutationMetric.h"
#include <stdarg.h>

enum MetricType
{
	METRIC_NONE, METRIC_EUCLIDEAN, METRIC_CLOSEST_PERMUTATION
};

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

