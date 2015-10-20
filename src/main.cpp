/**
 * Author: rodrigo
 * 2015
 */
#include <stdlib.h>
#include <iostream>
#include <string>
#include <opencv2/core/core.hpp>
#include <pcl/io/pcd_io.h>
#include "clustering/ClosestPermutation.h"
#include "clustering/KMeans.h"
#include "descriptor/Calculator.h"
#include "descriptor/Extractor.h"
#include "descriptor/Hist.h"
#include "utils/Config.h"
#include "utils/Helper.h"
#include "factories/CloudFactory.h"
#include "io/Writer.h"

#define CONFIG_LOCATION "./config/config"

int main(int _argn, char **_argv)
{
	clock_t begin = clock();

	try
	{
		if (system("rm -rf ./output/*") != 0)
			std::cout << "WARNING: can't clean output folder\n";

		std::cout << "Loading configuration file\n";
		if (!Config::load(CONFIG_LOCATION, _argn, _argv))
			throw std::runtime_error("Can't read configuration file at " CONFIG_LOCATION);

		ExecutionParams params = Config::getExecutionParams();
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>());

		// Load cloud
		if (!Helper::loadCloud(cloud, params))
			throw std::runtime_error("Can't load cloud");
		std::cout << "Loaded " << cloud->size() << " points in cloud\n";

		if (params.normalExecution)
		{
			std::cout << "Target point: " << params.targetPoint << "\n";

			pcl::PointNormal target = cloud->at(params.targetPoint);
			std::vector<BandPtr> bands = Calculator::calculateDescriptor(cloud, target, params);

			// Calculate histograms
			std::cout << "Generating angle histograms\n";
			std::vector<Hist> histograms;
			Calculator::calculateAngleHistograms(bands, histograms, params.useProjection);

			// Write output
			std::cout << "Writing output\n";
			Writer::writeOuputData(cloud, bands, histograms, params);
		}
		else
		{
			std::cout << "Execution for clustering\n";

			size_t cloudSize = cloud->size();
			size_t sequenceSize = params.getSequenceLength();
			cv::Mat descriptors = cv::Mat::zeros(cloudSize, sequenceSize * params.bandNumber, CV_32FC1);

			if (!Helper::loadClusteringCache(descriptors, params))
			{
				std::cout << "Cache not found, calculating descriptors\n";

				// Extract the descriptors
				for (size_t i = 0; i < cloudSize; i++)
				{
					pcl::PointNormal target = cloud->points[i];
					std::vector<BandPtr> bands = Calculator::calculateDescriptor(cloud, target, params);

					for (size_t j = 0; j < bands.size(); j++)
						memcpy(&descriptors.at<float>(i, j * sequenceSize), &bands[j]->sequenceVector[0], sizeof(float) * sequenceSize);
				}

				// Save cache to file
				Helper::writeClusteringCache(descriptors, params);
			}

			// Create an 'elbow criterion' graph using kmeans
			if (params.showElbow)
				Helper::generateElbowGraph(descriptors, params);

			std::cout << "Calculating clusters\n";

			// Make clusters of data
			cv::Mat labels, centers;
			//if (!Helper::loadClusters(centers, labels, params))
			{
				switch (params.implementation)
				{
					case CLUSTERING_OPENCV:
						cv::kmeans(descriptors, params.clusters, labels, cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, params.maxIterations, params.stopThreshold), params.attempts, cv::KMEANS_PP_CENTERS, centers);
						break;

					case CLUSTERING_CUSTOM:
						KMeans::searchClusters(descriptors, params.clusters, ClosestPermutation(sequenceSize), params.attempts, params.maxIterations, params.stopThreshold, labels, centers);
						break;

					case CLUSTERING_STOCHASTIC:
						KMeans::stochasticSearchClusters(descriptors, params.clusters, cloud->size() / 10, ClosestPermutation(sequenceSize), params.attempts, params.maxIterations, params.stopThreshold, labels, centers);
						break;

					default:
						break;
				}
			}

			pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr clusteredCloud = Helper::generateClusterRepresentation(cloud, labels, centers, params);
			pcl::io::savePCDFileASCII("./output/visualization.pcd", *clusteredCloud);

			// Color the data according to the clusters
			pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr colored = CloudFactory::createColorCloud(cloud, Helper::getColor(0));
			for (int i = 0; i < labels.rows; i++)
				(*colored)[i].rgb = Helper::getColor(labels.at<int>(i));
			pcl::io::savePCDFileASCII("./output/clusters.pcd", *colored);
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
