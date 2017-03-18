/**
 * Author: rodrigo
 * 2016
 */
#include <boost/test/unit_test.hpp>
#include "MetricFactory.hpp"


/** Dummy function to evaluate exceptions */
bool dummyCritical(std::runtime_error const& ex)
{
	return true;
}


/**************************************************/
BOOST_AUTO_TEST_SUITE(MetricFactory_class_suite)

BOOST_AUTO_TEST_CASE(toType)
{
	BOOST_CHECK_EQUAL(Metric::toType("euclidean"), METRIC_EUCLIDEAN);
	BOOST_CHECK_EQUAL(Metric::toType("closest"), METRIC_CLOSEST_PERMUTATION);
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

BOOST_AUTO_TEST_CASE(distanceWithDescriptors)
{
	// Metric to test DCH computed with 'mean' stat with 4 bins
	std::vector<std::string> args = std::vector<std::string>();
	args.push_back("");
	args.push_back("4");
	MetricPtr metric1 = MetricFactory::create(METRIC_CLOSEST_PERMUTATION, args);

	// 8 bands of size 4 => length 32
	cv::Mat vector1 = cv::Mat::ones(1, 32, CV_32FC1);
	cv::Mat vector2 = cv::Mat::ones(1, 32, CV_32FC1);
	BOOST_CHECK_CLOSE(0, metric1->distance(vector1, vector2), 0.01);


	/**********/


	// Metric to test DCH computed with 'hist10'
	args = std::vector<std::string>();
	args.push_back("");
	args.push_back("18");
	MetricPtr metric2 = MetricFactory::create(METRIC_CLOSEST_PERMUTATION, args);

	// 8 bands of size 18 => length 144
	vector1 = cv::Mat::ones(1, 144, CV_32FC1);
	vector2 = cv::Mat::ones(1, 144, CV_32FC1);
	BOOST_CHECK_CLOSE(0, metric2->distance(vector1, vector2), 0.01);


	vector1 = cv::Mat::zeros(1, 144, CV_32FC1);
	vector2 = cv::Mat::zeros(1, 144, CV_32FC1);

	vector1.at<float>(18) = 1;
	vector1.at<float>(19) = 1;
	vector1.at<float>(20) = 1;
	vector1.at<float>(21) = 2;
	vector1.at<float>(22) = 2;
	vector1.at<float>(23) = 2;
	vector1.at<float>(24) = 3;
	vector1.at<float>(25) = 3;
	vector1.at<float>(26) = 3;
	vector1.at<float>(27) = 4;
	vector1.at<float>(28) = 4;
	vector1.at<float>(29) = 4;
	vector1.at<float>(30) = 5;
	vector1.at<float>(31) = 5;
	vector1.at<float>(32) = 5;
	vector1.at<float>(33) = 6;
	vector1.at<float>(34) = 6;
	vector1.at<float>(35) = 6;

	vector2.at<float>(126) = 1;
	vector2.at<float>(127) = 1;
	vector2.at<float>(128) = 1;
	vector2.at<float>(129) = 2;
	vector2.at<float>(130) = 2;
	vector2.at<float>(131) = 2;
	vector2.at<float>(132) = 3;
	vector2.at<float>(133) = 3;
	vector2.at<float>(134) = 3;
	vector2.at<float>(135) = 4;
	vector2.at<float>(136) = 4;
	vector2.at<float>(137) = 4;
	vector2.at<float>(138) = 5;
	vector2.at<float>(139) = 5;
	vector2.at<float>(140) = 5;
	vector2.at<float>(141) = 6;
	vector2.at<float>(142) = 6;
	vector2.at<float>(143) = 6;

	BOOST_CHECK_CLOSE(0, metric2->distance(vector1, vector2), 0.01);

	/**********/


	// Metric to test DCH computed with 'hist20'
	args = std::vector<std::string>();
	args.push_back("");
	args.push_back("9");
	MetricPtr metric3 = MetricFactory::create(METRIC_CLOSEST_PERMUTATION, args);

	// 8 bands of size 9 => length 72
	vector1 = cv::Mat::ones(1, 72, CV_32FC1);
	vector2 = cv::Mat::ones(1, 72, CV_32FC1);
	BOOST_CHECK_CLOSE(0, metric3->distance(vector1, vector2), 0.01);


	vector1 = cv::Mat::zeros(1, 72, CV_32FC1);
	vector2 = cv::Mat::zeros(1, 72, CV_32FC1);

	vector1.at<float>(9)  = 1;
	vector1.at<float>(10) = 1;
	vector1.at<float>(11) = 1;
	vector1.at<float>(12) = 2;
	vector1.at<float>(13) = 2;
	vector1.at<float>(14) = 2;
	vector1.at<float>(15) = 3;
	vector1.at<float>(16) = 3;
	vector1.at<float>(17) = 3;

	vector2.at<float>(45) = 1;
	vector2.at<float>(46) = 1;
	vector2.at<float>(47) = 1;
	vector2.at<float>(48) = 2;
	vector2.at<float>(49) = 2;
	vector2.at<float>(50) = 2;
	vector2.at<float>(51) = 3;
	vector2.at<float>(52) = 3;
	vector2.at<float>(53) = 3;
	BOOST_CHECK_CLOSE(0, metric3->distance(vector1, vector2), 0.01);
}

BOOST_AUTO_TEST_SUITE_END()
/**************************************************/

/**************************************************/
BOOST_AUTO_TEST_SUITE(ClosestPermutationWithConfidenceMetric_class_suite)

BOOST_AUTO_TEST_CASE(testExceptions)
{
	std::vector<std::string> args = std::vector<std::string>();
	args.push_back("");
	args.push_back("1");

	MetricPtr metricPtr = MetricFactory::create(METRIC_CLOSEST_PERMUTATION_WITH_CONFIDENCE, args);
	ClosestPermutationWithConfidenceMetric *metric = (ClosestPermutationWithConfidenceMetric *)metricPtr.get();

	BOOST_CHECK_EXCEPTION(metric->distance(cv::Mat(), cv::Mat()), std::runtime_error, dummyCritical);
	BOOST_CHECK_EXCEPTION(metric->getClosestPermutation(cv::Mat(), cv::Mat()), std::runtime_error, dummyCritical);
	BOOST_CHECK_EXCEPTION(metric->calculateMeans(1, cv::Mat(), cv::Mat()), std::runtime_error, dummyCritical);
}

BOOST_AUTO_TEST_SUITE_END()
/**************************************************/