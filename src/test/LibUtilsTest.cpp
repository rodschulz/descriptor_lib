#include <boost/test/unit_test.hpp>

#include "../utils/Utils.hpp"
#include "../utils/ExecutionParams.hpp"

/**************************************************/
BOOST_AUTO_TEST_SUITE(Utils_class_suite)

BOOST_AUTO_TEST_CASE(num2Hex)
{
	std::string hex1 = Utils::num2Hex(0);
	BOOST_CHECK_EQUAL(hex1, "0");

	std::string hex2 = Utils::num2Hex(1234567890);
	BOOST_CHECK_EQUAL(hex2, "499602d2");

	std::string hex3 = Utils::num2Hex(-2957323069);
	BOOST_CHECK_EQUAL(hex3, "ffffffff4fbad4c3");
}

BOOST_AUTO_TEST_CASE(getColor)
{
	float value = 2.05454233E-38;
	float color = Utils::getColor(223, 184, 72);
	BOOST_CHECK_MESSAGE(abs(color - value) < 1E-90, "Conversion RGB to float failling");
}

BOOST_AUTO_TEST_CASE(getSSE)
{
	int nvectors = 10;
	int ncenters = 3;
	int dim = 5;

	cv::Mat vectors = cv::Mat::zeros(nvectors, dim, CV_32F);
	cv::Mat centers = cv::Mat::zeros(ncenters, dim, CV_32FC1);
	cv::Mat labels = cv::Mat::zeros(nvectors, 1, CV_32SC1);

	BOOST_CHECK_EQUAL(Utils::getSSE(vectors, centers, labels), 0);

	for (int i = 0; i < nvectors; i++)
	{
		labels.at<int>(i) = i % ncenters;

		cv::Mat aux = cv::Mat::ones(1, dim, CV_32FC1) * (i % ncenters);
		aux.copyTo(vectors.row(i));
	}

	int center1 = 7;
	int center2 = 2;
	int center3 = 3;

	cv::Mat aux = cv::Mat::ones(1, dim, CV_32FC1) * center1;
	aux.copyTo(centers.row(0));

	aux = cv::Mat::ones(1, dim, CV_32FC1) * center2;
	aux.copyTo(centers.row(1));

	aux = cv::Mat::ones(1, dim, CV_32FC1) * center3;
	aux.copyTo(centers.row(2));

	double sse = center1 * center1 * dim * 4 + (center2 - 1) * (center2 - 1) * dim * 3 + (center3 - 2) * (center3 - 2) * dim * 3;
	BOOST_CHECK_CLOSE(Utils::getSSE(vectors, centers, labels), sse, 1E-5);
}


BOOST_AUTO_TEST_CASE(sign)
{
	BOOST_CHECK_EQUAL(Utils::sign<short>(10), 1);
	BOOST_CHECK_EQUAL(Utils::sign<short>(-10), -1);
	BOOST_CHECK_EQUAL(Utils::sign<short>(0), 0);

	BOOST_CHECK_EQUAL(Utils::sign<int>(10), 1);
	BOOST_CHECK_EQUAL(Utils::sign<int>(-10), -1);
	BOOST_CHECK_EQUAL(Utils::sign<int>(0), 0);

	BOOST_CHECK_EQUAL(Utils::sign<long>(10), 1);
	BOOST_CHECK_EQUAL(Utils::sign<long>(-10), -1);
	BOOST_CHECK_EQUAL(Utils::sign<long>(0), 0);

	BOOST_CHECK_EQUAL(Utils::sign<double>(10), 1);
	BOOST_CHECK_EQUAL(Utils::sign<double>(-10), -1);
	BOOST_CHECK_EQUAL(Utils::sign<double>(0), 0);

	BOOST_CHECK_EQUAL(Utils::sign<float>(10), 1);
	BOOST_CHECK_EQUAL(Utils::sign<float>(-10), -1);
	BOOST_CHECK_EQUAL(Utils::sign<float>(0), 0);
}

BOOST_AUTO_TEST_CASE(angle)
{
	Eigen::Vector3f v1(1, 0, 0);
	Eigen::Vector3f v2(1, 0, 0);

	BOOST_CHECK_CLOSE(Utils::angle<Eigen::Vector3f>(v1, v2), 0, 1E-10);

	v1 = Eigen::Vector3f(1, 0, 0);
	v2 = Eigen::Vector3f(0, 1, 0);
	BOOST_CHECK_CLOSE(Utils::angle<Eigen::Vector3f>(v1, v2), M_PI_2, 1E-10);

	v1 = Eigen::Vector3f(0, 1, 0);
	v2 = Eigen::Vector3f(1, 0, 0);
	BOOST_CHECK_CLOSE(Utils::angle<Eigen::Vector3f>(v1, v2), M_PI_2, 1E-10);

	v1 = Eigen::Vector3f(1, 0, 0);
	v2 = Eigen::Vector3f(-1, 0, 0);
	BOOST_CHECK_CLOSE(Utils::angle<Eigen::Vector3f>(v1, v2), M_PI, 1E-10);

	v1 = Eigen::Vector3f(-1, 0, 0);
	v2 = Eigen::Vector3f(1, 0, 0);
	BOOST_CHECK_CLOSE(Utils::angle<Eigen::Vector3f>(v1, v2), M_PI, 1E-10);

	v1 = Eigen::Vector3f(1, 0, 0);
	v2 = Eigen::Vector3f(1, 1, 0);
	BOOST_CHECK_CLOSE(Utils::angle<Eigen::Vector3f>(v1, v2), M_PI_4, 1E-10);

	v1 = Eigen::Vector3f(1, 0, 0);
	v2 = Eigen::Vector3f(1, -1, 0);
	BOOST_CHECK_CLOSE(Utils::angle<Eigen::Vector3f>(v1, v2), M_PI_4, 1E-10);
}

BOOST_AUTO_TEST_CASE(signedAngle)
{
	Eigen::Vector3f v1(1, 0, 0);
	Eigen::Vector3f v2(1, 0, 0);
	Eigen::Vector3f normal(0, 0, 1);

	BOOST_CHECK_CLOSE(Utils::signedAngle<Eigen::Vector3f>(v1, v2, normal), 0, 1E-10);

	v1 = Eigen::Vector3f(1, 0, 0);
	v2 = Eigen::Vector3f(0, 1, 0);
	normal = Eigen::Vector3f(0, 0, 1);
	BOOST_CHECK_CLOSE(Utils::signedAngle<Eigen::Vector3f>(v1, v2, normal), M_PI_2, 1E-10);

	v1 = Eigen::Vector3f(1, 0, 0);
	v2 = Eigen::Vector3f(0, -1, 0);
	normal = Eigen::Vector3f(0, 0, 1);
	BOOST_CHECK_CLOSE(Utils::signedAngle<Eigen::Vector3f>(v1, v2, normal), -M_PI_2, 1E-10);

	v1 = Eigen::Vector3f(1, 0, 0);
	v2 = Eigen::Vector3f(1, 1, 0);
	normal = Eigen::Vector3f(0, 0, 1);
	BOOST_CHECK_CLOSE(Utils::signedAngle<Eigen::Vector3f>(v1, v2, normal), M_PI_4, 1E-10);

	v1 = Eigen::Vector3f(1, 0, 0);
	v2 = Eigen::Vector3f(1, -1, 0);
	normal = Eigen::Vector3f(0, 0, 1);
	BOOST_CHECK_CLOSE(Utils::signedAngle<Eigen::Vector3f>(v1, v2, normal), -M_PI_4, 1E-10);

	v1 = Eigen::Vector3f(1, 0, 0);
	v2 = Eigen::Vector3f(-1, 0, 0);
	normal = Eigen::Vector3f(0, 0, 1);
	BOOST_CHECK_CLOSE(Utils::signedAngle<Eigen::Vector3f>(v1, v2, normal), M_PI, 1E-10);
}

BOOST_AUTO_TEST_SUITE_END()
/**************************************************/

/**************************************************/
BOOST_AUTO_TEST_SUITE(ExecParams_class_suite)

BOOST_AUTO_TEST_CASE(constructor)
{
	BOOST_CHECK_MESSAGE(sizeof(ExecutionParams) == 200, "ExecutionParams size has changed, check that the new member is being properly initialized in the constructor");

	ExecutionParams params;

	BOOST_CHECK_EQUAL(params.executionType, EXECUTION_NONE);
	BOOST_CHECK_EQUAL(params.inputLocation, "");
	BOOST_CHECK_EQUAL(params.targetPoint, 1000);

	BOOST_CHECK_EQUAL(params.patchSize, 0.05);
	BOOST_CHECK_EQUAL(params.normalEstimationRadius, -1);
	BOOST_CHECK_EQUAL(params.bandNumber, 4);
	BOOST_CHECK_EQUAL(params.bandWidth, 0.01);
	BOOST_CHECK_EQUAL(params.bidirectional, true);
	BOOST_CHECK_EQUAL(params.useProjection, true);

	BOOST_CHECK_EQUAL(params.sequenceBin, 0.01);
	BOOST_CHECK_EQUAL(params.sequenceStat, STAT_NONE);

	BOOST_CHECK_EQUAL(params.useSynthetic, false);
	BOOST_CHECK_EQUAL(params.synCloudType, CLOUD_NONE);

	BOOST_CHECK_EQUAL(params.smoothingType, SMOOTHING_NONE);
	BOOST_CHECK_EQUAL(params.gaussianSigma, 2);
	BOOST_CHECK_EQUAL(params.gaussianRadius, 0.02);
	BOOST_CHECK_EQUAL(params.mlsRadius, 0.02);

	BOOST_CHECK_EQUAL(params.genElbowCurve, false);
	BOOST_CHECK_EQUAL(params.genDistanceMatrix, false);

	BOOST_CHECK_EQUAL(params.labelData, false);
	BOOST_CHECK_EQUAL(params.centersLocation, "");

	BOOST_CHECK_EQUAL(params.implementation, CLUSTERING_NONE);
	BOOST_CHECK_EQUAL(params.metric, METRIC_NONE);
	BOOST_CHECK_EQUAL(params.clusters, 5);
	BOOST_CHECK_EQUAL(params.maxIterations, 10000);
	BOOST_CHECK_EQUAL(params.stopThreshold, 0.001);
	BOOST_CHECK_EQUAL(params.attempts, 1);
	BOOST_CHECK_EQUAL(params.cacheLocation, "");
	BOOST_CHECK_EQUAL(params.useConfidence, false);

	BOOST_CHECK_EQUAL(params.targetMetric, METRIC_NONE);
	BOOST_CHECK_EQUAL(params.metricArgs.empty(), true);
}

BOOST_AUTO_TEST_CASE(getExecutionType)
{
	BOOST_CHECK_EQUAL(ExecutionParams::getExecutionType("descriptor"), EXECUTION_DESCRIPTOR);
	BOOST_CHECK_EQUAL(ExecutionParams::getExecutionType("clustering"), EXECUTION_CLUSTERING);
	BOOST_CHECK_EQUAL(ExecutionParams::getExecutionType("metric"), EXECUTION_METRIC);
	BOOST_CHECK_EQUAL(ExecutionParams::getExecutionType(""), EXECUTION_NONE);
}

BOOST_AUTO_TEST_CASE(getSynCloudType)
{
	BOOST_CHECK_EQUAL(ExecutionParams::getSynCloudType("cube"), CLOUD_CUBE);
	BOOST_CHECK_EQUAL(ExecutionParams::getSynCloudType("cylinder"), CLOUD_CYLINDER);
	BOOST_CHECK_EQUAL(ExecutionParams::getSynCloudType("sphere"), CLOUD_SPHERE);
	BOOST_CHECK_EQUAL(ExecutionParams::getSynCloudType(""), CLOUD_NONE);
}

BOOST_AUTO_TEST_CASE(getSmoothingType)
{
	BOOST_CHECK_EQUAL(ExecutionParams::getSmoothingType("gaussian"), SMOOTHING_GAUSSIAN);
	BOOST_CHECK_EQUAL(ExecutionParams::getSmoothingType("mls"), SMOOTHING_MLS);
	BOOST_CHECK_EQUAL(ExecutionParams::getSmoothingType(""), SMOOTHING_NONE);
}

BOOST_AUTO_TEST_CASE(getStatType)
{
	BOOST_CHECK_EQUAL(ExecutionParams::getStatType("mean"), STAT_MEAN);
	BOOST_CHECK_EQUAL(ExecutionParams::getStatType("median"), STAT_MEDIAN);
	BOOST_CHECK_EQUAL(ExecutionParams::getStatType(""), STAT_NONE);
}

BOOST_AUTO_TEST_CASE(getClusteringImplementation)
{
	BOOST_CHECK_EQUAL(ExecutionParams::getClusteringImplementation("opencv"), CLUSTERING_OPENCV);
	BOOST_CHECK_EQUAL(ExecutionParams::getClusteringImplementation("custom"), CLUSTERING_CUSTOM);
	BOOST_CHECK_EQUAL(ExecutionParams::getClusteringImplementation("stochastic"), CLUSTERING_STOCHASTIC);
	BOOST_CHECK_EQUAL(ExecutionParams::getClusteringImplementation(""), CLUSTERING_NONE);
}

BOOST_AUTO_TEST_CASE(getMetricType)
{
	BOOST_CHECK_EQUAL(ExecutionParams::getMetricType("euclidean"), METRIC_EUCLIDEAN);
	BOOST_CHECK_EQUAL(ExecutionParams::getMetricType("closest"), METRIC_CLOSEST_PERMUTATION);
	BOOST_CHECK_EQUAL(ExecutionParams::getMetricType(""), METRIC_NONE);
}

BOOST_AUTO_TEST_CASE(getBandsAngularRange)
{
	ExecutionParams params;

	params.bidirectional = true;
	BOOST_CHECK_EQUAL(params.getBandsAngularRange(), M_PI);
	params.bidirectional = false;
	BOOST_CHECK_EQUAL(params.getBandsAngularRange(), 2 * M_PI);
}

BOOST_AUTO_TEST_CASE(getBandsAngularStep)
{
	ExecutionParams params;

	params.bidirectional = true;
	params.bandNumber = 10;
	BOOST_CHECK_EQUAL(params.getBandsAngularStep(), M_PI / 10);

	params.bidirectional = false;
	params.bandNumber = 10;
	BOOST_CHECK_EQUAL(params.getBandsAngularStep(), M_PI / 5);
}

BOOST_AUTO_TEST_CASE(getSequenceLength)
{
	ExecutionParams params;

	params.bidirectional = true;
	params.patchSize = 10;
	params.sequenceBin = 2;
	BOOST_CHECK_EQUAL(params.getSequenceLength(), 10);

	params.bidirectional = false;
	params.patchSize = 10;
	params.sequenceBin = 2;
	BOOST_CHECK_EQUAL(params.getSequenceLength(), 5);
}

BOOST_AUTO_TEST_SUITE_END()
/**************************************************/
