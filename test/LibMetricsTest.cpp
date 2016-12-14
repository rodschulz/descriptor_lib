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
BOOST_AUTO_TEST_SUITE(MetricFactory_class_suite)

BOOST_AUTO_TEST_CASE(toMetricType)
{
	BOOST_CHECK_EQUAL(Metric::toMetricType("euclidean"), METRIC_EUCLIDEAN);
	BOOST_CHECK_EQUAL(Metric::toMetricType("closest"), METRIC_CLOSEST_PERMUTATION);
}

BOOST_AUTO_TEST_SUITE_END()
/**************************************************/

/**************************************************/
BOOST_AUTO_TEST_SUITE(EuclideanMetric_class_suite)

BOOST_AUTO_TEST_CASE(euclideanDistance)
{
	MetricPtr metric = MetricFactory::create(METRIC_EUCLIDEAN);

	cv::Mat vector1 = cv::Mat::ones(3, 1, CV_32FC1);
	cv::Mat vector2 = cv::Mat::ones(3, 1, CV_32FC1);
	BOOST_CHECK_CLOSE(0, metric->distance(vector1, vector2), 0.01);

	vector1.at<float>(0) = 10;
	vector1.at<float>(1) = -3;
	vector1.at<float>(2) = 2;

	vector2.at<float>(0) = 0;
	vector2.at<float>(1) = 7;
	vector2.at<float>(2) = 7;

	BOOST_CHECK_CLOSE(15, metric->distance(vector1, vector2), 0.01);
}

BOOST_AUTO_TEST_SUITE_END()
/**************************************************/

/**************************************************/
BOOST_AUTO_TEST_SUITE(ClosestPermutationMetric_class_suite)

BOOST_AUTO_TEST_CASE(closestDistance)
{
	std::vector<std::string> args = std::vector<std::string>();
	args.push_back("");
	args.push_back("1");
	MetricPtr metric = MetricFactory::create(METRIC_CLOSEST_PERMUTATION, args);

	cv::Mat vector1 = cv::Mat::ones(1, 5, CV_32FC1);
	cv::Mat vector2 = cv::Mat::ones(1, 5, CV_32FC1);
	BOOST_CHECK_CLOSE(0, metric->distance(vector1, vector2), 0.01);

	vector1.at<float>(0) = 8;
	vector2.at<float>(3) = 0;
	BOOST_CHECK_CLOSE(sqrt(50), metric->distance(vector1, vector2), 0.01);

	vector1.at<float>(0) = 8;
	vector2.at<float>(0) = 0;
	vector2.at<float>(3) = 1;
	BOOST_CHECK_CLOSE(sqrt(50), metric->distance(vector1, vector2), 0.01);

	args = std::vector<std::string>();
	args.push_back("");
	args.push_back("2");
	metric = MetricFactory::create(METRIC_CLOSEST_PERMUTATION, args);

	vector1 = cv::Mat::zeros(1, 8, CV_32FC1);
	vector2 = cv::Mat::zeros(1, 8, CV_32FC1);

	vector1.at<float>(0) = 1;
	vector1.at<float>(1) = 1;
	vector1.at<float>(2) = 2;
	vector1.at<float>(3) = 2;
	vector1.at<float>(4) = 3;
	vector1.at<float>(5) = 3;
	vector1.at<float>(6) = 4;
	vector1.at<float>(7) = 4;

	vector2.at<float>(0) = 1;
	vector2.at<float>(1) = 1;
	vector2.at<float>(2) = 0;
	vector2.at<float>(3) = 0;
	vector2.at<float>(4) = 1;
	vector2.at<float>(5) = 1;
	vector2.at<float>(6) = 0;
	vector2.at<float>(7) = 0;

	BOOST_CHECK_CLOSE(sqrt(40), metric->distance(vector1, vector2), 0.01);
}

BOOST_AUTO_TEST_CASE(closestPermutation)
{
	std::vector<std::string> args = std::vector<std::string>();
	args.push_back("");
	args.push_back("1");
	ClosestPermutationMetric *metric = (ClosestPermutationMetric *)MetricFactory::create(METRIC_CLOSEST_PERMUTATION, args).get();

	cv::Mat vector1 = cv::Mat::ones(1, 5, CV_32FC1);
	cv::Mat vector2 = cv::Mat::ones(1, 5, CV_32FC1);

	vector1.at<float>(0) = 8;
	vector2.at<float>(3) = 0;
	ClosestPermutationMetric::Permutation permutation = metric->getClosestPermutation(vector1, vector2);
	BOOST_CHECK_CLOSE(sqrt(50), permutation.distance, 0.01);
	BOOST_CHECK_EQUAL(0, permutation.index);

	vector1.at<float>(0) = 8;
	vector2.at<float>(0) = 0;
	vector2.at<float>(3) = 1;
	permutation = metric->getClosestPermutation(vector1, vector2);
	BOOST_CHECK_CLOSE(sqrt(50), permutation.distance, 0.01);
	BOOST_CHECK_EQUAL(1, permutation.index);

	args = std::vector<std::string>();
	args.push_back("");
	args.push_back("2");
	metric = (ClosestPermutationMetric *)MetricFactory::create(METRIC_CLOSEST_PERMUTATION, args).get();

	vector1 = cv::Mat::zeros(1, 8, CV_32FC1);
	vector2 = cv::Mat::zeros(1, 8, CV_32FC1);

	vector1.at<float>(0) = 1;
	vector1.at<float>(1) = 1;
	vector1.at<float>(2) = 2;
	vector1.at<float>(3) = 2;
	vector1.at<float>(4) = 3;
	vector1.at<float>(5) = 3;
	vector1.at<float>(6) = 4;
	vector1.at<float>(7) = 4;

	vector2.at<float>(0) = 1;
	vector2.at<float>(1) = 1;
	vector2.at<float>(2) = 0;
	vector2.at<float>(3) = 0;
	vector2.at<float>(4) = 1;
	vector2.at<float>(5) = 1;
	vector2.at<float>(6) = 0;
	vector2.at<float>(7) = 0;

	permutation = metric->getClosestPermutation(vector1, vector2);
	BOOST_CHECK_CLOSE(sqrt(40), permutation.distance, 0.01);
	BOOST_CHECK_EQUAL(1, permutation.index);
}

BOOST_AUTO_TEST_SUITE_END()
/**************************************************/
