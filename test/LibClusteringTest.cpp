/**
 * Author: rodrigo
 * 2016
 */
#include <boost/test/unit_test.hpp>
#include "Clustering.hpp"
#include "ClusteringUtils.hpp"
#include "KMeans.hpp"
#include "MetricFactory.hpp"

/**************************************************/
BOOST_AUTO_TEST_SUITE(Clustering_class_suite)
BOOST_AUTO_TEST_SUITE_END()
/**************************************************/

/**************************************************/
BOOST_AUTO_TEST_SUITE(KMeans_class_suite)

BOOST_AUTO_TEST_CASE(labelDataMetric)
{
	cv::Mat data = cv::Mat::zeros(11, 2, CV_32FC1);
	// Center 2
	data.at<float>(0, 0) = 4;
	data.at<float>(0, 1) = 3;
	data.at<float>(3, 0) = 3;
	data.at<float>(3, 1) = 3;
	data.at<float>(5, 0) = 4;
	data.at<float>(5, 1) = 2;
	data.at<float>(7, 0) = 5;
	data.at<float>(7, 1) = 4;
	data.at<float>(10, 0) = 5;
	data.at<float>(10, 1) = 4;

	// Center 0
	data.at<float>(1, 0) = -1;
	data.at<float>(1, 1) = 2;
	data.at<float>(2, 0) = -2;
	data.at<float>(2, 1) = 1;
	data.at<float>(9, 0) = -2;
	data.at<float>(9, 1) = 2;
	data.at<float>(4, 0) = -4;
	data.at<float>(4, 1) = 4;

	// Center 1
	data.at<float>(6, 0) = 2;
	data.at<float>(6, 1) = -3;
	data.at<float>(8, 0) = 3;
	data.at<float>(8, 1) = -3;

	// Centers for labeling
	cv::Mat centers = cv::Mat::zeros(3, 2, CV_32FC1);
	centers.at<float>(0, 0) = -2.25;
	centers.at<float>(0, 1) = 2.25;

	centers.at<float>(1, 0) = 2.5;
	centers.at<float>(1, 1) = -3;

	centers.at<float>(2, 0) = 4.2;
	centers.at<float>(2, 1) = 3.2;

	cv::Mat labels;
	MetricPtr metric = MetricFactory::createMetric(METRIC_EUCLIDEAN, std::vector<std::string>());
	ClusteringUtils::labelData(data, centers, metric, labels);

	BOOST_CHECK_EQUAL(labels.at<int>(0), 2);
	BOOST_CHECK_EQUAL(labels.at<int>(1), 0);
	BOOST_CHECK_EQUAL(labels.at<int>(2), 0);
	BOOST_CHECK_EQUAL(labels.at<int>(3), 2);
	BOOST_CHECK_EQUAL(labels.at<int>(4), 0);
	BOOST_CHECK_EQUAL(labels.at<int>(5), 2);
	BOOST_CHECK_EQUAL(labels.at<int>(6), 1);
	BOOST_CHECK_EQUAL(labels.at<int>(7), 2);
	BOOST_CHECK_EQUAL(labels.at<int>(8), 1);
	BOOST_CHECK_EQUAL(labels.at<int>(9), 0);
	BOOST_CHECK_EQUAL(labels.at<int>(10), 2);
}

// BOOST_AUTO_TEST_CASE(labelDataSVM)
// {
// 	// Centers for labeling
// 	cv::Mat center = cv::Mat::zeros(3, 2, CV_32FC1);
// 	center.at<float>(0, 0) = -2.25;
// 	center.at<float>(0, 1) = 2.25;

// 	center.at<float>(1, 0) = 2.5;
// 	center.at<float>(1, 1) = -3;

// 	center.at<float>(2, 0) = 4.2;
// 	center.at<float>(2, 1) = 3.2;

// 	std::map<std::string, std::string> metadata;
// 	metadata["metric"] = metricType[METRIC_EUCLIDEAN];
// 	SVMPtr svm = ClusteringUtils::prepareClassifier(center, metadata);

// 	cv::Mat data = cv::Mat::zeros(11, 2, CV_32FC1);
// 	// Center 2
// 	data.at<float>(0, 0) = 4;
// 	data.at<float>(0, 1) = 3;
// 	data.at<float>(3, 0) = 3;
// 	data.at<float>(3, 1) = 3;
// 	data.at<float>(5, 0) = 4;
// 	data.at<float>(5, 1) = 2;
// 	data.at<float>(7, 0) = 5;
// 	data.at<float>(7, 1) = 4;
// 	data.at<float>(10, 0) = 5;
// 	data.at<float>(10, 1) = 4;

// 	// Center 0
// 	data.at<float>(1, 0) = -1;
// 	data.at<float>(1, 1) = 2;
// 	data.at<float>(2, 0) = -2;
// 	data.at<float>(2, 1) = 1;
// 	data.at<float>(9, 0) = -2;
// 	data.at<float>(9, 1) = 2;
// 	data.at<float>(4, 0) = -4;
// 	data.at<float>(4, 1) = 4;

// 	// Center 1
// 	data.at<float>(6, 0) = 2;
// 	data.at<float>(6, 1) = -3;
// 	data.at<float>(8, 0) = 3;
// 	data.at<float>(8, 1) = -3;

// 	cv::Mat labels;
// 	ClusteringUtils::labelData(data, svm, labels);

// 	BOOST_CHECK_EQUAL(labels.at<float>(0), 3);
// 	BOOST_CHECK_EQUAL(labels.at<float>(1), 1);
// 	BOOST_CHECK_EQUAL(labels.at<float>(2), 1);
// 	BOOST_CHECK_EQUAL(labels.at<float>(3), 3);
// 	BOOST_CHECK_EQUAL(labels.at<float>(4), 1);
// 	BOOST_CHECK_EQUAL(labels.at<float>(5), 3);
// 	BOOST_CHECK_EQUAL(labels.at<float>(6), 2);
// 	BOOST_CHECK_EQUAL(labels.at<float>(7), 3);
// 	BOOST_CHECK_EQUAL(labels.at<float>(8), 2);
// 	BOOST_CHECK_EQUAL(labels.at<float>(9), 1);
// 	BOOST_CHECK_EQUAL(labels.at<float>(10), 3);
// }

BOOST_AUTO_TEST_SUITE_END()
/**************************************************/

/**************************************************/
BOOST_AUTO_TEST_SUITE(KMeans_class_suite)

BOOST_AUTO_TEST_CASE(kmeansSearchClusters)
{
	cv::Mat data = cv::Mat::zeros(11, 2, CV_32FC1);
	// Center 2
	data.at<float>(0, 0) = 4;
	data.at<float>(0, 1) = 3;
	data.at<float>(3, 0) = 3;
	data.at<float>(3, 1) = 3;
	data.at<float>(5, 0) = 4;
	data.at<float>(5, 1) = 2;
	data.at<float>(7, 0) = 5;
	data.at<float>(7, 1) = 4;
	data.at<float>(10, 0) = 5;
	data.at<float>(10, 1) = 4;

	// Center 0
	data.at<float>(1, 0) = -1;
	data.at<float>(1, 1) = 2;
	data.at<float>(2, 0) = -2;
	data.at<float>(2, 1) = 1;
	data.at<float>(9, 0) = -2;
	data.at<float>(9, 1) = 2;
	data.at<float>(4, 0) = -4;
	data.at<float>(4, 1) = 4;

	// Center 1
	data.at<float>(6, 0) = 2;
	data.at<float>(6, 1) = -3;
	data.at<float>(8, 0) = 3;
	data.at<float>(8, 1) = -3;

	MetricPtr metric = MetricFactory::createMetric(METRIC_EUCLIDEAN);
	ClusteringResults results;

	// Temporarily disable std out printing
	std::cout.setstate(std::ios_base::failbit);
	KMeans::searchClusters(results, data, metric, 3, 5, 100, 0.01);
	std::cout.clear();

	// Sort the centers to check them
	cv::Mat firstCol = results.centers.col(0);
	cv::Mat1i index;
	cv::sortIdx(firstCol, index, cv::SORT_EVERY_COLUMN + cv::SORT_ASCENDING);
	cv::Mat centers (results.centers.rows, results.centers.cols, results.centers.type());
	for (int i = 0; i < centers.rows; i++)
		results.centers.row(i).copyTo(centers.row(index(0, i)));

	BOOST_CHECK_CLOSE(centers.row(0).at<float>(0, 0), -2.25, 0.1);
	BOOST_CHECK_CLOSE(centers.row(0).at<float>(0, 1), 2.25, 0.1);
	BOOST_CHECK_CLOSE(centers.row(1).at<float>(0, 0), 2.5, 0.1);
	BOOST_CHECK_CLOSE(centers.row(1).at<float>(0, 1), -3, 0.1);
	BOOST_CHECK_CLOSE(centers.row(2).at<float>(0, 0), 4.2, 0.1);
	BOOST_CHECK_CLOSE(centers.row(2).at<float>(0, 1), 3.2, 0.1);

	// Sort labels
	std::vector<int> itemCount(results.centers.rows);
	std::vector<int> newLabels(results.labels.rows);
	for (int i = 0; i < results.labels.rows; i++)
	{
		itemCount[index(0, results.labels.at<int>(0, i))]++;
		newLabels[i] = index(0, results.labels.at<int>(0, i));
	}

	// Check item's are correctly labeled
	BOOST_CHECK_EQUAL(itemCount[0], 4);
	BOOST_CHECK_EQUAL(itemCount[1], 2);
	BOOST_CHECK_EQUAL(itemCount[2], 5);

	// 2 0 0 2 0 2 1 2 1 0 2
	BOOST_CHECK_EQUAL(newLabels[0], 2);
	BOOST_CHECK_EQUAL(newLabels[1], 0);
	BOOST_CHECK_EQUAL(newLabels[2], 0);
	BOOST_CHECK_EQUAL(newLabels[3], 2);
	BOOST_CHECK_EQUAL(newLabels[4], 0);
	BOOST_CHECK_EQUAL(newLabels[5], 2);
	BOOST_CHECK_EQUAL(newLabels[6], 1);
	BOOST_CHECK_EQUAL(newLabels[7], 2);
	BOOST_CHECK_EQUAL(newLabels[8], 1);
	BOOST_CHECK_EQUAL(newLabels[9], 0);
	BOOST_CHECK_EQUAL(newLabels[10], 2);
}

BOOST_AUTO_TEST_CASE(kmeansSSE)
{
	cv::Mat data = cv::Mat::zeros(11, 2, CV_32FC1);
	// Center 2
	data.at<float>(0, 0) = 4;
	data.at<float>(0, 1) = 3;
	data.at<float>(3, 0) = 3;
	data.at<float>(3, 1) = 3;
	data.at<float>(5, 0) = 4;
	data.at<float>(5, 1) = 2;
	data.at<float>(7, 0) = 5;
	data.at<float>(7, 1) = 4;
	data.at<float>(10, 0) = 5;
	data.at<float>(10, 1) = 4;

	// Center 0
	data.at<float>(1, 0) = -1;
	data.at<float>(1, 1) = 2;
	data.at<float>(2, 0) = -2;
	data.at<float>(2, 1) = 1;
	data.at<float>(9, 0) = -2;
	data.at<float>(9, 1) = 2;
	data.at<float>(4, 0) = -4;
	data.at<float>(4, 1) = 4;

	// Center 1
	data.at<float>(6, 0) = 2;
	data.at<float>(6, 1) = -3;
	data.at<float>(8, 0) = 3;
	data.at<float>(8, 1) = -3;

	// 2 0 0 2 0 2 1 2 1 0 2
	cv::Mat labels = cv::Mat::zeros(11, 1, CV_32SC1);
	labels.at<int>(0) = 2;
	labels.at<int>(1) = 0;
	labels.at<int>(2) = 0;
	labels.at<int>(3) = 2;
	labels.at<int>(4) = 0;
	labels.at<int>(5) = 2;
	labels.at<int>(6) = 1;
	labels.at<int>(7) = 2;
	labels.at<int>(8) = 1;
	labels.at<int>(9) = 0;
	labels.at<int>(10) = 2;

	cv::Mat centers = cv::Mat::zeros(3, 2, CV_32FC1);
	centers.at<float>(0, 0) = -2.25;
	centers.at<float>(0, 1) = 2.25;
	centers.at<float>(1, 0) = 2.5;
	centers.at<float>(1, 1) = -3;
	centers.at<float>(2, 0) = 4.2;
	centers.at<float>(2, 1) = 3.2;

	double sse = ClusteringUtils::getSSE(data, labels, centers, MetricFactory::createMetric(METRIC_EUCLIDEAN));

	BOOST_CHECK_CLOSE(15.6, sse, 0.1);
}

BOOST_AUTO_TEST_SUITE_END()
/**************************************************/
