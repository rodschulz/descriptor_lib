/**
 * Author: rodrigo
 * 2015
 */
#include "Helper.h"
//#include <pcl/io/pcd_io.h>
#include <Eigen/Geometry>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include <ctype.h>
#include <stdexcept>
#include "ExecutionParams.h"
#include "../factories/CloudFactory.h"
#include "../factories/PointFactory.h"
#include "../descriptor/Calculator.h"

#include "Utils.hpp"
#include "CloudUtils.hpp"

Helper::Helper()
{
}

Helper::~Helper()
{
}

//bool Helper::loadCloud(pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const ExecutionParams &_params)
//{
//	bool loadOk = true;
//
//	// Get cartesian data
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ(new pcl::PointCloud<pcl::PointXYZ>());
//	if (!_params.useSynthetic)
//	{
//		if (pcl::io::loadPCDFile<pcl::PointXYZ>(_params.inputLocation, *cloudXYZ) != 0)
//		{
//			std::cout << "ERROR: Can't read file from disk (" << _params.inputLocation << ")\n";
//			loadOk = false;
//		}
//
//		switch (_params.smoothingType)
//		{
//			case SMOOTHING_GAUSSIAN:
//				std::cout << "Applying gaussian smoothing\n";
//				cloudXYZ = CloudUtils::gaussianSmoothing(cloudXYZ, _params.gaussianSigma, _params.gaussianRadius);
//				break;
//
//			case SMOOTHING_MLS:
//				std::cout << "Applying MLS smoothing\n";
//				cloudXYZ = CloudUtils::MLSSmoothing(cloudXYZ, _params.mlsRadius);
//				break;
//
//			default:
//				break;
//		}
//	}
//	else
//	{
//		switch (_params.synCloudType)
//		{
//			case CLOUD_CUBE:
//				CloudFactory::createCube(0.3, PointFactory::createPointXYZ(0.3, 0.3, 0.3), cloudXYZ);
//				break;
//
//			case CLOUD_CYLINDER:
//				CloudFactory::createCylinder(0.2, 0.5, PointFactory::createPointXYZ(0.4, 0.4, 0.4), cloudXYZ);
//				break;
//
//			case CLOUD_SPHERE:
//				CloudFactory::createSphere(0.2, PointFactory::createPointXYZ(0.2, 0.2, 0.2), cloudXYZ);
//				break;
//
//			default:
//				cloudXYZ->clear();
//				loadOk = false;
//				std::cout << "WARNING, wrong cloud generation parameters\n";
//		}
//	}
//
//	// Estimate normals
//	if (loadOk)
//	{
//		CloudUtils::removeNANs(cloudXYZ);
//		pcl::PointCloud<pcl::Normal>::Ptr normals = CloudUtils::estimateNormals(cloudXYZ, _params.normalEstimationRadius);
//
//		_cloud->clear();
//		pcl::concatenateFields(*cloudXYZ, *normals, *_cloud);
//	}
//
//	return loadOk;
//}

void Helper::generateDescriptorsCache(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const ExecutionParams &_params, cv::Mat &_descriptors)
{
	std::cout << "Generating descriptors cache\n";
	int sequenceSize = _params.getSequenceLength();

	// Resize the matrix in case it doesn't match the required dimensions
	int rows = _cloud->size();
	int cols = sequenceSize * _params.bandNumber;
	if (_descriptors.rows != rows || _descriptors.cols != cols)
		_descriptors = cv::Mat::zeros(rows, cols, CV_32FC1);

	// Extract the descriptors
	for (size_t i = 0; i < _cloud->size(); i++)
	{
		pcl::PointNormal target = _cloud->points[i];
		std::vector<BandPtr> bands = Calculator::calculateDescriptor(_cloud, target, _params);

		for (size_t j = 0; j < bands.size(); j++)
			memcpy(&_descriptors.at<float>(i, j * sequenceSize), &bands[j]->sequenceVector[0], sizeof(float) * sequenceSize);
	}
}



void Helper::generateElbowGraph(const cv::Mat &_descriptors, const ExecutionParams &_params)
{
	int attempts = 5;
	cv::Mat labels, centers;

	std::ofstream data;
	data.open("./output/graph.dat", std::fstream::out);
	for (int i = 2; i < 50; i++)
	{
		cv::kmeans(_descriptors, i, labels, cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, _params.maxIterations, _params.stopThreshold), attempts, cv::KMEANS_PP_CENTERS, centers);
		double sse = Utils::getSSE(_descriptors, centers, labels);
		data << i << " " << sse << "\n";
	}
	data.close();

	std::ofstream graph;
	graph.open("./output/graph.plot", std::fstream::out);

	graph << "set xtic auto\n";
	graph << "set ytic auto\n";
	graph << "set grid ytics xtics\n\n";

	graph << "set title 'Sum of Squared Errors'\n";
	graph << "set xlabel 'Number of Clusters'\n";
	graph << "set ylabel 'SSE'\n\n";

	graph << "set term png\n";
	graph << "set output './output/elbowGraph.png'\n\n";

	graph << "plot './output/graph.dat' using 1:2 title 'SSE' with linespoints lt rgb 'blue'\n";

	graph.close();

	if (system("gnuplot ./output/graph.plot") != 0)
		std::cout << "WARNING: can't execute GNUPlot" << std::endl;
}

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr Helper::generateClusterRepresentation(const pcl::PointCloud<pcl::PointNormal>::Ptr _cloud, const cv::Mat &_labels, const cv::Mat &_centers, const ExecutionParams &_params)
{
	int sequenceLength = _params.getSequenceLength();
	std::vector<pcl::PointNormal> locations(_centers.rows, PointFactory::createPointNormal(0, 0, 0, 0, 0, 0, 0));
	std::vector<int> pointsPerCluster(_centers.rows, 0);

	for (size_t i = 0; i < locations.size(); i++)
	{
		locations[i].x = _params.patchSize;
		locations[i].z = _params.patchSize * 3 * i;
	}

	// Angular step between bands
	double bandAngularStep = _params.getBandsAngularStep();

	// Define reference vectors
	Eigen::Vector3f referenceNormal = Eigen::Vector3f(1, 0, 0);
	Eigen::Vector3f referenceRotationAxis = Eigen::Vector3f(0, 1, 0);

	// Create the vectors placed according to the cluster centroid info
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
	for (int i = 0; i < _centers.rows; i++)
	{
		for (int j = 0; j < _params.bandNumber; j++)
		{
			Eigen::Vector3f baseLocation = locations[i].getVector3fMap();

			// Calculate the normal's rotation axis according to the current band's angle
			double bandAngle = bandAngularStep * j;
			Eigen::Vector3f rotationAxis = Eigen::AngleAxis<float>(bandAngle, referenceNormal).matrix() * referenceRotationAxis;
			rotationAxis.normalize();

			for (int k = 0; k < sequenceLength; k++)
			{
				// Calculate the rotated normal according to the angle of the current bin in the current band
				float angle = _centers.at<float>(i, j * sequenceLength + k);
				Eigen::Vector3f rotatedNormal = Eigen::AngleAxis<float>(angle, rotationAxis).matrix() * referenceNormal;
				rotatedNormal.normalize();

				// Create a plane oriented according to the rotated normal vector
				Eigen::Hyperplane<float, 3> plane;
				if (output->empty())
					plane = Eigen::Hyperplane<float, 3>(rotatedNormal, baseLocation);
				else
					plane = Eigen::Hyperplane<float, 3>(rotatedNormal, output->back().getVector3fMap());

				// Add 9 points to represent the bin
				for (double sideStep = -_params.bandWidth / 2; sideStep < _params.bandWidth; sideStep += _params.bandWidth / 2)
				{
					for (double forwardStep = 0; forwardStep < _params.sequenceBin; forwardStep += _params.sequenceBin / 3)
					{
						float x = 0;
						float y = sideStep;
						float z = forwardStep + k * _params.sequenceBin;
						Eigen::Vector3f displaced = Eigen::Vector3f(x, y, z);
						Eigen::Vector3f point = Eigen::AngleAxis<float>(bandAngle, referenceNormal) * displaced;
						point = point + Eigen::Vector3f(baseLocation.x(), baseLocation.y(), baseLocation.z());

						// Project the point over a plane oriented according the normal of the band's bin
						//Eigen::Vector3f p = plane.projection(point);
						Eigen::Vector3f p = point;

						float nx = rotatedNormal.x();
						float ny = rotatedNormal.y();
						float nz = rotatedNormal.z();

						output->push_back(PointFactory::createPointXYZRGBNormal(p.x(), p.y(), p.z(), nx, ny, nz, 0, Utils::getColor(i)));
					}
				}
			}
		}
	}

	return output;
}

void Helper::evaluateMetricCases(const std::string &_resultsFilename, const std::string &_testcasesFilename, const MetricType &_metricType, const std::vector<std::string> &_args)
{
	std::cout << "Evaluating metric testcases" << std::endl;
	MetricPtr targetMetric;
	if (_metricType == METRIC_EUCLIDEAN)
		targetMetric = MetricPtr(new EuclideanMetric());
	else if (_metricType == METRIC_CLOSEST_PERMUTATION)
		targetMetric = MetricPtr(new ClosestPermutationMetric(atoi(_args[0].c_str()), _args[1].compare("true") == 0));
	else
		throw std::runtime_error("Invalid metric type");

	// Extract the testcases
	boost::property_tree::ptree tree;
	boost::property_tree::read_json(_testcasesFilename, tree);

	std::cout << "Test cases loaded" << std::endl;

	int k = 0;
	bool shift = true;
	std::vector<std::vector<float> > data1, data2;
	BOOST_FOREACH (boost::property_tree::ptree::value_type& testCase, tree.get_child("vectors"))
	{
		// Generate arrays for the vectors of this testcase
		data1.push_back(std::vector<float>());
		data2.push_back(std::vector<float>());

		// Iterate over the array of vectors
		BOOST_FOREACH (boost::property_tree::ptree::value_type& vectors, testCase.second)
		{
			// Extract data for a vector
			BOOST_FOREACH (boost::property_tree::ptree::value_type& vector, vectors.second)
			{
				float value = vector.second.get_value<float>();
				shift ? data1[k].push_back(value) : data2[k].push_back(value);
			}

			shift = !shift;
		}
		k++;
	}

	std::cout << "Evaluating metric" << std::endl;

	// Evaluate each testcase and write the results
	std::ofstream results;
	results.open(_resultsFilename.c_str(), std::ofstream::out);
	for (size_t i = 0; i < data1.size(); i++)
	{
		cv::Mat vector1 = cv::Mat(data1[i]).t();
		cv::Mat vector2 = cv::Mat(data2[i]).t();
		double distance = targetMetric->distance(vector1, vector2);

		results << vector1 << "\n" << vector2 << "\ndistance = " << distance << "\n\n";
	}
	results.close();

	std::cout << "Metric evaluated" << std::endl;
}
