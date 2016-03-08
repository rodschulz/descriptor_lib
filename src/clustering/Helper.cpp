/**
 * Author: rodrigo
 * 2015
 */
#include "Helper.h"
//#include <pcl/io/pcd_io.h>
#include <Eigen/Geometry>
//#include <boost/property_tree/ptree.hpp>
//#include <boost/property_tree/json_parser.hpp>
//#include <boost/foreach.hpp>
#include <ctype.h>
#include <stdexcept>
#include "../factories/CloudFactory.hpp"
#include "../factories/PointFactory.hpp"
#include "../utils/ExecutionParams.hpp"

#include <string.h>
#include <sstream>
#include <iostream>
#include <fstream>

#include "../descriptor/Calculator.hpp"
#include "../utils/Utils.hpp"
#include "../utils/CloudUtils.hpp"

Helper::Helper()
{
}

Helper::~Helper()
{
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
