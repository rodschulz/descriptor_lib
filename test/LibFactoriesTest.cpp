/**
 * Author: rodrigo
 * 2016
 */
#include <boost/test/unit_test.hpp>
#include "MetricFactory.hpp"

/**************************************************/
BOOST_AUTO_TEST_SUITE(MetricFactory_class_suite)

BOOST_AUTO_TEST_CASE(create)
{
	// Creation of Euclidean metric
	MetricPtr metric = MetricFactory::create(METRIC_EUCLIDEAN);
	BOOST_CHECK(metric.get() != NULL);

	EuclideanMetric* euclidean = dynamic_cast<EuclideanMetric*> (metric.get());
	BOOST_CHECK(euclidean != NULL);

	// Creation of ClosestPermutation metric
	std::vector<std::string> args = std::vector<std::string>();
	args.push_back("");
	args.push_back("4");
	metric = MetricFactory::create(METRIC_CLOSEST_PERMUTATION, args);
	BOOST_CHECK(metric.get() != NULL);

	ClosestPermutationMetric* closestPermutation = dynamic_cast<ClosestPermutationMetric*> (metric.get());
	BOOST_CHECK(closestPermutation != NULL);

	// Creation of ClosestPermutationWithConfidence metric
	metric = MetricFactory::create(METRIC_CLOSEST_PERMUTATION_WITH_CONFIDENCE, args);
	BOOST_CHECK(metric.get() != NULL);

	ClosestPermutationWithConfidenceMetric* closestPermutationWithConfidence = dynamic_cast<ClosestPermutationWithConfidenceMetric*> (metric.get());
	BOOST_CHECK(closestPermutationWithConfidence != NULL);
}

BOOST_AUTO_TEST_SUITE_END()
/**************************************************/

/**************************************************/
BOOST_AUTO_TEST_SUITE(PointFactory_class_suite)
BOOST_AUTO_TEST_SUITE_END()
/**************************************************/

/**************************************************/
BOOST_AUTO_TEST_SUITE(CloudFactory_class_suite)
BOOST_AUTO_TEST_SUITE_END()
/**************************************************/
