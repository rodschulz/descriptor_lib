/**
 * Author: rodrigo
 * 2016
 */
#include <boost/test/unit_test.hpp>
#include "../descriptor/Calculator.hpp"
#include "../descriptor/Extractor.hpp"
#include "../factories/CloudFactory.hpp"

// Fixture definition
struct ExecParamsFixture {
	ExecParamsFixture()
	{
		params.targetPoint = 100;
		params.patchSize = 5;
		params.normalEstimationRadius = -1;
		params.bandNumber = 5;
		params.bandWidth = 2;
		params.bidirectional = false;
		params.useProjection = false;
		params.sequenceBin = 2;
		params.sequenceStat = STAT_MEAN;
	}
	~ ExecParamsFixture() {}

	ExecutionParams params;
};

/**************************************************/
BOOST_AUTO_TEST_SUITE(Calculator_class_suite)

BOOST_FIXTURE_TEST_CASE(calculateDescriptor, ExecParamsFixture)
{
	// Generate cloud
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud = CloudFactory::createHorizontalPlane(-50, 50, 200, 300, 30, 3000);
	pcl::PointNormal point = cloud->at(params.targetPoint);

	Descriptor descriptor1 = Calculator::calculateDescriptor(cloud, params);
	Descriptor descriptor2 = Calculator::calculateDescriptor(cloud, params, point);

	// Check sizes
	BOOST_CHECK_EQUAL(descriptor1.size(), params.bandNumber);
	BOOST_CHECK_EQUAL(descriptor1.size(), descriptor2.size());

	// Check the fields are being filled
	for (int i = 0; i < params.bandNumber; i++)
	{
		BOOST_CHECK(!descriptor1[i]->sequenceString.empty());
		BOOST_CHECK(!descriptor1[i]->sequenceVector.empty());
	}
}

BOOST_FIXTURE_TEST_CASE(fillSequences, ExecParamsFixture)
{
	// Generate cloud
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud = CloudFactory::createHorizontalPlane(-50, 50, 200, 300, 30, 5000);
	pcl::PointNormal point = cloud->at(1);
	point.x = 0;
	point.y = 250;

	// Extract bands
	std::vector<BandPtr> bands = Extractor::getBands(cloud, point, params);
	Calculator::fillSequences(bands, params, M_PI / 18);

	int sequenceSize = params.getSequenceLength();
	std::string zeroSequence = "";
	zeroSequence.resize(sequenceSize, '0');
	for (int i = 0; i < params.bandNumber; i++)
	{
		BOOST_CHECK_EQUAL(bands[i]->sequenceVector.size(), sequenceSize);
		BOOST_CHECK_EQUAL(bands[i]->sequenceString.size(), sequenceSize);
		BOOST_CHECK_EQUAL(bands[i]->sequenceString, zeroSequence);
	}
}

BOOST_AUTO_TEST_CASE(getSequenceChar)
{
	BOOST_CHECK_EQUAL(Calculator::getSequenceChar(1, 5), '0');
	BOOST_CHECK_EQUAL(Calculator::getSequenceChar(-1, 5), '0');

	BOOST_CHECK_EQUAL(Calculator::getSequenceChar(7, 5), 'A');
	BOOST_CHECK_EQUAL(Calculator::getSequenceChar(12, 5), 'B');
	BOOST_CHECK_EQUAL(Calculator::getSequenceChar(19, 5), 'C');

	BOOST_CHECK_EQUAL(Calculator::getSequenceChar(-7, 5), 'a');
	BOOST_CHECK_EQUAL(Calculator::getSequenceChar(-12, 5), 'b');
	BOOST_CHECK_EQUAL(Calculator::getSequenceChar(-19, 5), 'c');
}

BOOST_AUTO_TEST_SUITE_END()
/**************************************************/

/**************************************************/
BOOST_AUTO_TEST_SUITE(Extractor_class_suite)

BOOST_FIXTURE_TEST_CASE(getBands_no_bidirectional, ExecParamsFixture)
{
	params.bidirectional = false;

	// Generate cloud
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud = CloudFactory::createHorizontalPlane(-50, 50, 200, 300, 30, 3000);
	pcl::PointNormal point = cloud->at(params.targetPoint);

	// Extract bands
	std::vector<BandPtr> bands = Extractor::getBands(cloud, point, params);

	// Check number of bands is ok
	BOOST_CHECK_EQUAL(bands.size(), params.bandNumber);

	// Check bands are ok
	Eigen::Vector3f pointNormal = point.getNormalVector3fMap();
	for (int i = 0; i < params.bandNumber; i++)
	{
		Eigen::Vector3f normal = bands[i]->plane.normal();
		Eigen::Vector3f nextNormal = bands[(i + 1) % params.bandNumber]->plane.normal();

		// Check the angular difference is less than a 0.5 percent
		BOOST_CHECK_CLOSE(Utils::angle(normal, nextNormal), params.getBandsAngularStep(), 0.5);
		// Check the angular difference is less than 0.005 radians (0.28 degrees aprox)
		BOOST_CHECK_SMALL(fabs(Utils::angle(normal, nextNormal) - params.getBandsAngularStep()), 5E-3);

		// Check each band's plane is perpendicular (goes along splitting the band in two)
		BOOST_CHECK_SMALL(pointNormal.dot(normal), 1E-10f);
	}
}

BOOST_FIXTURE_TEST_CASE(getBands_bidirectional, ExecParamsFixture)
{
	params.bidirectional = true;

	// Generate cloud
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud = CloudFactory::createHorizontalPlane(-50, 50, 200, 300, 30, 3000);
	pcl::PointNormal point = cloud->at(params.targetPoint);

	// Extract bands
	std::vector<BandPtr> bands = Extractor::getBands(cloud, point, params);

	// Check number of bands is ok
	BOOST_CHECK_EQUAL(bands.size(), params.bandNumber);

	// Extract bands
	bands = Extractor::getBands(cloud, point, params);

	// Check bands are ok
	Eigen::Vector3f pointNormal = point.getNormalVector3fMap();
	for (int i = 0; i < params.bandNumber; i++)
	{
		Eigen::Vector3f normal = bands[i]->plane.normal();
		Eigen::Vector3f nextNormal = bands[(i + 1) % params.bandNumber]->plane.normal();

		float step = i != params.bandNumber - 1 ? params.getBandsAngularStep() : M_PI - params.getBandsAngularStep();

		// Check the angular difference is less than a 0.5 percent
		BOOST_CHECK_CLOSE(Utils::angle(normal, nextNormal), step, 0.5);
		// Check the angular difference is less than 0.005 radians (0.28 degrees aprox)
		BOOST_CHECK_SMALL(fabs(Utils::angle(normal, nextNormal) - step), 5E-3);

		// Check each band's plane is perpendicular (goes along splitting the band in two)
		BOOST_CHECK_SMALL(pointNormal.dot(normal), 1E-10f);
	}
}

BOOST_AUTO_TEST_SUITE_END()