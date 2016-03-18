/**
 * Author: rodrigo
 * 2015
 */
#include <stdlib.h>
#include <iostream>
#include <string>
#include <opencv2/core/core.hpp>
#include <pcl/io/pcd_io.h>
#include "clustering/Clustering.hpp"
#include "clustering/KMeans.hpp"
#include "clustering/MetricFactory.hpp"
#include "descriptor/Calculator.hpp"
#include "descriptor/Extractor.hpp"
#include "descriptor/Hist.hpp"
#include "factories/CloudFactory.hpp"
#include "io/Loader.hpp"
#include "io/Writer.hpp"
#include "utils/Config.hpp"
#include "utils/CloudUtils.hpp"

#define CONFIG_LOCATION "./config/config.yaml"

bool getPointCloud(pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const ExecutionParams &_params)
{
	if (_params.useSynthetic)
	{
		std::cout << "Generating synthetic cloud: " << _params.synCloudType << "\n";

		switch (_params.synCloudType)
		{
			case CLOUD_CUBE:
				_cloud = CloudFactory::createCube(5, Eigen::Vector3f(0, 0, 0), 20000);
				break;

			case CLOUD_CYLINDER:
				_cloud = CloudFactory::createCylinderSection(2 * M_PI, 5, 10, Eigen::Vector3f(0, 0, 0), 20000);
				break;

			case CLOUD_SPHERE:
				_cloud = CloudFactory::createSphereSection(2 * M_PI, 10, Eigen::Vector3f(0, 0, 0), 20000);
				break;

			case CLOUD_HALF_SPHERE:
				_cloud = CloudFactory::createSphereSection(M_PI, 10, Eigen::Vector3f(0, 0, 0), 20000);
				break;

			case CLOUD_PLANE:
				_cloud = CloudFactory::createHorizontalPlane(-50, 50, 200, 300, 30, 20000);
				break;

			default:
				std::cout << "WARNING, wrong cloud generation parameters\n";
				return false;
		}

		return true;
	}
	else
	{
		std::cout << "Loading point cloud\n";
		bool result = Loader::loadCloud(_cloud, _params);
		std::cout << "Loaded " << _cloud->size() << " points in cloud\n";

		return result;
	}
}

int main(int _argn, char **_argv)
{
	clock_t begin = clock();

	try
	{
		if (system("rm -rf ./output/*") != 0)
			std::cout << "WARNING: can't clean output folder\n";

		std::cout << "Loading configuration file\n";
		if (!Config::load(CONFIG_LOCATION))
			throw std::runtime_error("Problem reading configuration file at " CONFIG_LOCATION);
		ExecutionParams params = Config::getExecutionParams();

		// Check if enough params were given
		if (_argn < 2 && !params.useSynthetic)
		{
			throw std::runtime_error("Not enough exec params given\nUsage: Descriptor <input_file>");
			params.inputLocation = _argv[1];
		}

		// Do things according to the execution type
		if (params.executionType == EXECUTION_METRIC)
			Metric::evaluateMetricCases("./output/metricEvaluation", params.inputLocation, params.targetMetric, params.metricArgs);
		else
		{
			// Load cloud
			pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>());
			if (!getPointCloud(cloud, params))
				throw std::runtime_error("Can't load cloud");

			// Select execution type
			if (params.executionType == EXECUTION_DESCRIPTOR)
			{
				std::cout << "...Execution for descriptor calculation\n";

				std::cout << "Target point: " << params.targetPoint << "\n";
				Descriptor descriptor = Calculator::calculateDescriptor(cloud, params);

				// Calculate histograms
				std::cout << "Generating histograms\n";
				std::vector<Hist> histograms = Calculator::generateAngleHistograms(descriptor, params.useProjection);

				// Write output
				std::cout << "Writing output\n";
				Writer::writeOuputData(cloud, descriptor, histograms, params);
			}
			else if (params.executionType == EXECUTION_CLUSTERING)
			{
				std::cout << "...Execution for clustering\n";

				cv::Mat descriptors;
				if (!Loader::loadDescriptors(descriptors, params))
				{
					std::cout << "Cache not found, calculating descriptors\n";
					Calculator::calculateDescriptors(cloud, params, descriptors);
					Writer::writeDescriptorsCache(descriptors, params);
				}

				ClusteringResults results;
				MetricPtr metric = MetricFactory::createMetric(params.metric, params.getSequenceLength(), params.useConfidence);
				if (!params.labelData)
				{
					Clustering::searchClusters(descriptors, params, results);

					std::cout << "Generating SSE plot" << std::endl;
					Writer::writePlotSSE("sse", "SSE Evolution", results.errorEvolution);
				}
				else
				{
					std::cout << "Loading centers" << std::endl;

					if (!Loader::loadCenters(params.centersLocation, results.centers))
						throw std::runtime_error("Can't load clusters centers");

					Clustering::labelData(descriptors, results.centers, params, results.labels);
				}

				// Generate outputs
				std::cout << "Writing outputs" << std::endl;
				pcl::io::savePCDFileASCII("./output/visualization.pcd", *Clustering::generateClusterRepresentation(cloud, results.labels, results.centers, params));
				Writer::writeClusteredCloud("./output/clusters.pcd", cloud, results.labels);
				Writer::writeClustersCenters("./output/", results.centers);

				if (params.genDistanceMatrix)
					Writer::writeDistanceMatrix("./output/", descriptors, results.centers, results.labels, metric);
				if (params.genElbowCurve)
					Clustering::generateElbowGraph(descriptors, params);
			}
		}
	}
	catch (std::exception &_ex)
	{
		std::cout << "ERROR: " << _ex.what() << std::endl;
	}

	clock_t end = clock();
	double elapsedTime = double(end - begin) / CLOCKS_PER_SEC;

	std::cout << std::fixed << std::setprecision(3) << "Finished in " << elapsedTime << " [s]\n";
	return EXIT_SUCCESS;
}
