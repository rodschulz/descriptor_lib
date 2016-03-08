/**
 * Author: rodrigo
 * 2015
 */
#include <stdlib.h>
#include <iostream>
#include <string>
#include <opencv2/core/core.hpp>
#include <pcl/io/pcd_io.h>

#include "clustering/ClusteringUtils.hpp"
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
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ;
		switch (_params.synCloudType)
		{
			case CLOUD_CUBE:
				cloudXYZ = CloudFactory::createCube(0.3, PointFactory::createPointXYZ(0.3, 0.3, 0.3));
				break;

			case CLOUD_CYLINDER:
				cloudXYZ = CloudFactory::createCylinder(0.2, 0.5, PointFactory::createPointXYZ(0.4, 0.4, 0.4));
				break;

			case CLOUD_SPHERE:
				cloudXYZ = CloudFactory::createSphere(0.2, PointFactory::createPointXYZ(0.2, 0.2, 0.2));
				break;

			default:
				std::cout << "WARNING, wrong cloud generation parameters\n";
				return false;
		}

		pcl::PointCloud<pcl::Normal>::Ptr normals = CloudUtils::estimateNormals(cloudXYZ, _params.normalEstimationRadius);
		pcl::concatenateFields(*cloudXYZ, *normals, *_cloud);

		return true;
	}
	else
		return Loader::loadCloud(_cloud, _params);
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
			throw std::runtime_error("Not enough exec params given\nUsage: Descriptor <input_file>");
		params.inputLocation = _argv[1];

		// Do things according to the execution type
		if (params.executionType == EXECUTION_METRIC)
			Metric::evaluateMetricCases("./output/metricEvaluation", params.inputLocation, params.targetMetric, params.metricArgs);
		else
		{
			// Load cloud
			pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>());
			if (!getPointCloud(cloud, params))
				throw std::runtime_error("Can't load cloud");
			std::cout << "Loaded " << cloud->size() << " points in cloud\n";

			// Select execution type
			if (params.executionType == EXECUTION_DESCRIPTOR)
			{
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
				std::cout << "Execution for clustering\n";

				cv::Mat descriptors, labels, centers;
				if (!Loader::loadDescriptorsCache(descriptors, params))
				{
					Calculator::calculateDescriptors(cloud, params, descriptors);
					Writer::writeDescriptorsCache(descriptors, params);
				}

				std::vector<double> sseError;
				MetricPtr metric = MetricFactory::createMetric(params.metric, params.getSequenceLength(), params.useConfidence);
				if (!params.labelData)
				{
					// Make clusters of data
					std::cout << "Calculating clusters\n";
					if (params.implementation == CLUSTERING_OPENCV)
						cv::kmeans(descriptors, params.clusters, labels, cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, params.maxIterations, params.stopThreshold), params.attempts, cv::KMEANS_PP_CENTERS, centers);

					else if (params.implementation == CLUSTERING_CUSTOM)
						KMeans::searchClusters(descriptors, params.clusters, *metric, params.attempts, params.maxIterations, params.stopThreshold, labels, centers, sseError);

					else if (params.implementation == CLUSTERING_STOCHASTIC)
						KMeans::stochasticSearchClusters(descriptors, params.clusters, cloud->size() / 10, *metric, params.attempts, params.maxIterations, params.stopThreshold, labels, centers, sseError);

					if (!sseError.empty())
					{
						std::cout << "Generating SSE plot" << std::endl;
						Writer::writePlotSSE("sse", "SSE Evolution", sseError);
					}
				}
				else
				{
					std::cout << "Loading centers" << std::endl;

					// Label data according to given centers
					if (!Loader::loadClusterCenters(params.centersLocation, centers))
						throw std::runtime_error("Can't load clusters centers");

					labels = cv::Mat::zeros(descriptors.rows, 1, CV_32SC1);
					std::vector<double> distance(descriptors.rows, std::numeric_limits<double>::max());
					for (int i = 0; i < descriptors.rows; i++)
					{
						for (int j = 0; j < centers.rows; j++)
						{
							double dist = metric->distance(centers.row(j), descriptors.row(i));
							if (dist < distance[i])
							{
								distance[i] = dist;
								labels.at<int>(i) = j;
							}
						}
					}
				}

				// Generate outputs
				std::cout << "Writing outputs" << std::endl;
				pcl::io::savePCDFileASCII("./output/visualization.pcd", *ClusteringUtils::generateClusterRepresentation(cloud, labels, centers, params));
				Writer::writeClusteredCloud("./output/clusters.pcd", cloud, labels);
				Writer::writeClustersCenters("./output/", centers);

				if (params.genDistanceMatrix)
					Writer::writeDistanceMatrix("./output/", descriptors, centers, labels, metric);
				if (params.genElbowCurve)
					ClusteringUtils::generateElbowGraph(descriptors, params);
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
