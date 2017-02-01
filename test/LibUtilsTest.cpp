/**
 * Author: rodrigo
 * 2016
 */
#include <boost/test/unit_test.hpp>
#include <typeinfo>
#include "Utils.hpp"
#include "ExecutionParams.hpp"

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
	uint32_t value = 0x00DFB848;
	uint32_t color = Utils::getColor(223, 184, 72);
	BOOST_CHECK_EQUAL(color, value);
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
	Eigen::Vector3f v2(0.99, 0, 0);
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

BOOST_AUTO_TEST_CASE(generateAxes)
{
	Eigen::Vector3f normal = Eigen::Vector3f(1, 1, 0).normalized();
	Eigen::Hyperplane<float, 3> plane = Eigen::Hyperplane<float, 3>(normal, Eigen::Vector3f(10, 7, 5));

	Eigen::Vector3f origin = plane.projection(Eigen::Vector3f(-3, 2, 70));
	std::pair<Eigen::Vector3f, Eigen::Vector3f> axes = Utils::generateAxes(plane, origin);

	// Check vectors are actually perpendicular
	BOOST_CHECK_SMALL(axes.first.dot(axes.second), 1E-8f);

	// Check both are inside the plane
	BOOST_CHECK_CLOSE(axes.first.cross(axes.second).dot(normal), 1, 2);
	BOOST_CHECK_SMALL(axes.second.dot(normal), 1E-20f);
}

BOOST_AUTO_TEST_SUITE_END()
/**************************************************/

/**************************************************/
BOOST_AUTO_TEST_SUITE(ExecutionParams_suite)

BOOST_AUTO_TEST_CASE(strToSynCloudType)
{
	BOOST_CHECK_EQUAL(Params::toSynCloudType("cube"), Params::CLOUD_CUBE);
	BOOST_CHECK_EQUAL(Params::toSynCloudType("cylinder"), Params::CLOUD_CYLINDER);
	BOOST_CHECK_EQUAL(Params::toSynCloudType("sphere"), Params::CLOUD_SPHERE);
}

BOOST_AUTO_TEST_CASE(strToStatType)
{
	BOOST_CHECK_EQUAL(Params::toStatType("mean"), Params::STAT_MEAN);
	BOOST_CHECK_EQUAL(Params::toStatType("median"), Params::STAT_MEDIAN);
	BOOST_CHECK_EQUAL(Params::toStatType("hist10"), Params::STAT_HISTOGRAM_10);
	BOOST_CHECK_EQUAL(Params::toStatType("hist20"), Params::STAT_HISTOGRAM_20);
	BOOST_CHECK_EQUAL(Params::toStatType("hist30"), Params::STAT_HISTOGRAM_30);
	BOOST_CHECK_EQUAL(Params::toStatType("hb10"), Params::STAT_HISTOGRAM_BIN_10);
	BOOST_CHECK_EQUAL(Params::toStatType("hb20"), Params::STAT_HISTOGRAM_BIN_20);
	BOOST_CHECK_EQUAL(Params::toStatType("hb30"), Params::STAT_HISTOGRAM_BIN_30);
}

BOOST_AUTO_TEST_CASE(strToClusteringImp)
{
	BOOST_CHECK_EQUAL(Params::toClusteringImp("opencv"), Params::CLUSTERING_OPENCV);
	BOOST_CHECK_EQUAL(Params::toClusteringImp("kmeans"), Params::CLUSTERING_KMEANS);
	BOOST_CHECK_EQUAL(Params::toClusteringImp("stochastic"), Params::CLUSTERING_STOCHASTIC);
}

BOOST_AUTO_TEST_SUITE_END()
/**************************************************/

/**************************************************/
BOOST_AUTO_TEST_SUITE(DescriptorParams_class_suite)

BOOST_AUTO_TEST_CASE(DescriptorParams_toType)
{
	BOOST_CHECK_EQUAL(DescriptorParams::toType("DCH"), Params::DESCRIPTOR_DCH);
	BOOST_CHECK_EQUAL(DescriptorParams::toType("dch"), Params::DESCRIPTOR_DCH);
	BOOST_CHECK_EQUAL(DescriptorParams::toType("DESCRIPTOR_DCH"), Params::DESCRIPTOR_DCH);

	BOOST_CHECK_EQUAL(DescriptorParams::toType("SHOT"), Params::DESCRIPTOR_SHOT);
	BOOST_CHECK_EQUAL(DescriptorParams::toType("shot"), Params::DESCRIPTOR_SHOT);
	BOOST_CHECK_EQUAL(DescriptorParams::toType("DESCRIPTOR_SHOT"), Params::DESCRIPTOR_SHOT);

	BOOST_CHECK_EQUAL(DescriptorParams::toType("USC"), Params::DESCRIPTOR_USC);
	BOOST_CHECK_EQUAL(DescriptorParams::toType("usc"), Params::DESCRIPTOR_USC);
	BOOST_CHECK_EQUAL(DescriptorParams::toType("DESCRIPTOR_USC"), Params::DESCRIPTOR_USC);

	BOOST_CHECK_EQUAL(DescriptorParams::toType("PFH"), Params::DESCRIPTOR_PFH);
	BOOST_CHECK_EQUAL(DescriptorParams::toType("pfh"), Params::DESCRIPTOR_PFH);
	BOOST_CHECK_EQUAL(DescriptorParams::toType("DESCRIPTOR_PFH"), Params::DESCRIPTOR_PFH);

	BOOST_CHECK_EQUAL(DescriptorParams::toType("FPFH"), Params::DESCRIPTOR_FPFH);
	BOOST_CHECK_EQUAL(DescriptorParams::toType("fpfh"), Params::DESCRIPTOR_FPFH);
	BOOST_CHECK_EQUAL(DescriptorParams::toType("DESCRIPTOR_FPFH"), Params::DESCRIPTOR_FPFH);

	BOOST_CHECK_EQUAL(DescriptorParams::toType("ROPS"), Params::DESCRIPTOR_ROPS);
	BOOST_CHECK_EQUAL(DescriptorParams::toType("rops"), Params::DESCRIPTOR_ROPS);
	BOOST_CHECK_EQUAL(DescriptorParams::toType("DESCRIPTOR_ROPS"), Params::DESCRIPTOR_ROPS);

	BOOST_CHECK_EQUAL(DescriptorParams::toType("SpinImage"), Params::DESCRIPTOR_SPIN_IMAGE);
	BOOST_CHECK_EQUAL(DescriptorParams::toType("spinimage"), Params::DESCRIPTOR_SPIN_IMAGE);
	BOOST_CHECK_EQUAL(DescriptorParams::toType("DESCRIPTOR_SPIN_IMAGE"), Params::DESCRIPTOR_SPIN_IMAGE);

	BOOST_CHECK_EQUAL(DescriptorParams::toType("dummy"), Params::DESCRIPTOR_DCH);
}

BOOST_AUTO_TEST_CASE(DescriptorParams_create)
{
	DescriptorParamsPtr params1 = DescriptorParams::create(Params::DESCRIPTOR_DCH);
	DescriptorParamsPtr params2 = DescriptorParamsPtr(new DCHParams());
	BOOST_CHECK_EQUAL(typeid(*params1.get()).name(), typeid(*params2.get()).name());

	params1 = DescriptorParams::create(Params::DESCRIPTOR_SHOT);
	params2 = DescriptorParamsPtr(new SHOTParams());
	BOOST_CHECK_EQUAL(typeid(*params1.get()).name(), typeid(*params2.get()).name());

	params1 = DescriptorParams::create(Params::DESCRIPTOR_USC);
	params2 = DescriptorParamsPtr(new USCParams());
	BOOST_CHECK_EQUAL(typeid(*params1.get()).name(), typeid(*params2.get()).name());

	params1 = DescriptorParams::create(Params::DESCRIPTOR_PFH);
	params2 = DescriptorParamsPtr(new PFHParams());
	BOOST_CHECK_EQUAL(typeid(*params1.get()).name(), typeid(*params2.get()).name());

	params1 = DescriptorParams::create(Params::DESCRIPTOR_FPFH);
	params2 = DescriptorParamsPtr(new FPFHParams());
	BOOST_CHECK_EQUAL(typeid(*params1.get()).name(), typeid(*params2.get()).name());

	params1 = DescriptorParams::create(Params::DESCRIPTOR_ROPS);
	params2 = DescriptorParamsPtr(new ROPSParams());
	BOOST_CHECK_EQUAL(typeid(*params1.get()).name(), typeid(*params2.get()).name());

	params1 = DescriptorParams::create(Params::DESCRIPTOR_SPIN_IMAGE);
	params2 = DescriptorParamsPtr(new SpinImageParams());
	BOOST_CHECK_EQUAL(typeid(*params1.get()).name(), typeid(*params2.get()).name());

	params1 = DescriptorParams::create(Params::DESCRIPTOR_UNKNOWN);
	params2 = DescriptorParamsPtr(new DCHParams());
	BOOST_CHECK_EQUAL(typeid(*params1.get()).name(), typeid(*params2.get()).name());
}
BOOST_AUTO_TEST_SUITE_END()
/**************************************************/

/**************************************************/
BOOST_AUTO_TEST_SUITE(DCH_suite)

BOOST_AUTO_TEST_CASE(DCH_constructor)
{
	BOOST_CHECK_EQUAL(sizeof(DCHParams), 40);
	BOOST_CHECK_MESSAGE(sizeof(DCHParams) == 40, "DescriptorParams size changed, check that any new member is being properly initialized in the constructor");

	DCHParams params;
	BOOST_CHECK_CLOSE(params.searchRadius, 0.05, 1e-5);
	BOOST_CHECK_EQUAL(params.bandNumber, 4);
	BOOST_CHECK_CLOSE(params.bandWidth, 0.01, 1e-5);
	BOOST_CHECK_EQUAL(params.bidirectional, true);
	BOOST_CHECK_EQUAL(params.useProjection, true);
	BOOST_CHECK_EQUAL(params.binNumber, 1);
	BOOST_CHECK_EQUAL(params.stat, Params::STAT_MEAN);
}

BOOST_AUTO_TEST_CASE(DCHParams_bandsAngleRange)
{
	DCHParams params;

	params.bidirectional = true;
	BOOST_CHECK_CLOSE(params.bandsAngleRange(), M_PI, 1e-5);
	params.bidirectional = false;
	BOOST_CHECK_CLOSE(params.bandsAngleRange(), 2 * M_PI, 1e-5);
}

BOOST_AUTO_TEST_CASE(DCHParams_bandsAngleStep)
{
	DCHParams params;

	params.bidirectional = true;
	params.bandNumber = 10;
	BOOST_CHECK_CLOSE(params.bandsAngleStep(), M_PI / 10, 1e-5);

	params.bidirectional = false;
	params.bandNumber = 10;
	BOOST_CHECK_CLOSE(params.bandsAngleStep(), M_PI / 5, 1e-5);
}

BOOST_AUTO_TEST_CASE(DCHParams_sizePerBand)
{
	DCHParams params;

	params.bidirectional = true;
	params.searchRadius = 10;
	params.binNumber = 2;
	BOOST_CHECK_EQUAL(params.sizePerBand(), 2);
	params.bidirectional = false;
	params.searchRadius = 10;
	params.binNumber = 1;
	BOOST_CHECK_EQUAL(params.sizePerBand(), 1);

	params.stat = Params::STAT_HISTOGRAM_10;
	BOOST_CHECK_EQUAL(params.sizePerBand(), 18);

	params.stat = Params::STAT_HISTOGRAM_20;
	BOOST_CHECK_EQUAL(params.sizePerBand(), 9);

	params.stat = Params::STAT_HISTOGRAM_30;
	BOOST_CHECK_EQUAL(params.sizePerBand(), 6);
}
BOOST_AUTO_TEST_SUITE_END()
/**************************************************/

/**************************************************/
BOOST_AUTO_TEST_SUITE(ClusteringParams_class_suite)

BOOST_AUTO_TEST_CASE(constructor)
{
	BOOST_CHECK_EQUAL(sizeof(ClusteringParams), 48);
	BOOST_CHECK_MESSAGE(sizeof(ClusteringParams) == 48, "ClusteringParams size changed, check that any new member is being properly initialized in the constructor");

	ClusteringParams params;

	BOOST_CHECK_EQUAL(params.implementation, Params::CLUSTERING_OPENCV);
	BOOST_CHECK_EQUAL(params.metric, MetricPtr());
	BOOST_CHECK_EQUAL(params.clusterNumber, 5);
	BOOST_CHECK_EQUAL(params.maxIterations, 10000);
	BOOST_CHECK_EQUAL(params.stopThreshold, 0.001);
	BOOST_CHECK_EQUAL(params.attempts, 1);
	BOOST_CHECK_EQUAL(params.generateElbowCurve, false);
	BOOST_CHECK_EQUAL(params.generateDistanceMatrix, false);
}

BOOST_AUTO_TEST_SUITE_END()
/**************************************************/

/**************************************************/
BOOST_AUTO_TEST_SUITE(CloudSmoothingParams_class_suite)

BOOST_AUTO_TEST_CASE(constructor)
{
	BOOST_CHECK_EQUAL(sizeof(CloudSmoothingParams), 24);
	BOOST_CHECK_MESSAGE(sizeof(CloudSmoothingParams) == 24, "CloudSmoothingParams size changed, check that any new member is being properly initialized in the constructor");

	CloudSmoothingParams params;

	BOOST_CHECK_EQUAL(params.useSmoothing, false);
	BOOST_CHECK_EQUAL(params.sigma, 2);
	BOOST_CHECK_EQUAL(params.radius, 0.02);
}

BOOST_AUTO_TEST_SUITE_END()
/**************************************************/

/**************************************************/
BOOST_AUTO_TEST_SUITE(SyntheticCloudsParams_class_suite)

BOOST_AUTO_TEST_CASE(constructor)
{
	BOOST_CHECK_EQUAL(sizeof(SyntheticCloudsParams), 8);
	BOOST_CHECK_MESSAGE(sizeof(SyntheticCloudsParams) == 8, "SyntheticCloudsParams size changed, check that any new member is being properly initialized in the constructor");

	SyntheticCloudsParams params;

	BOOST_CHECK_EQUAL(params.useSynthetic, false);
	BOOST_CHECK_EQUAL(params.synCloudType, Params::CLOUD_SPHERE);
}

BOOST_AUTO_TEST_SUITE_END()
/**************************************************/
