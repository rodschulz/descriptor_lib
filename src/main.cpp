/**
 * Author: rodrigo
 * 2015
 */
#include <stdlib.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <string>
#include "Config.h"
#include "Helper.h"
#include "Extractor.h"
#include "Hist.h"
#include "Writer.h"
#include "Calculator.h"

#define CONFIG_LOCATION "./config/config"

void writeOuput(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const pcl::PointCloud<pcl::PointNormal>::Ptr &_patch, const std::vector<BandPtr> &_bands, const std::vector<Hist> &_angleHistograms, const ExecutionParams &_params)
{
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr coloredCloud = Helper::createColorCloud(_cloud, Helper::getColor(0));

	std::cout << "Writing clouds to disk\n";
	pcl::io::savePCDFileASCII("./output/cloud.pcd", *coloredCloud);
	pcl::io::savePCDFileASCII("./output/patch.pcd", *Helper::createColorCloud(_patch, Helper::getColor(11)));

	(*coloredCloud)[_params.targetPoint].rgb = Helper::getColor(255, 0, 0);
	pcl::io::savePCDFileASCII("./output/pointPosition.pcd", *coloredCloud);

	std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> planes = Extractor::getBandPlanes(_bands, _params);

	std::ofstream sequences;
	sequences.open("./output/sequences", std::fstream::out);

	for (size_t i = 0; i < _bands.size(); i++)
	{
		if (!_bands[i]->data->empty())
		{
			char name[100];
			sprintf(name, "./output/band%d.pcd", (int) i);
			pcl::io::savePCDFileASCII(name, *Helper::createColorCloud(_bands[i]->data, Helper::getColor(i + 1)));

			sequences << "band " << i << "\n";
			sequences << "\tmean  : " << _bands[i]->sequenceMean << "\n";
			sequences << "\tmedian: " << _bands[i]->sequenceMedian << "\n";

			sprintf(name, "./output/planeBand%d.pcd", (int) i);
			pcl::io::savePCDFileASCII(name, *planes[i]);
		}
	}

	sequences.close();

	// Write histogram data
	double limit = M_PI;
	Writer::writeHistogram("angles", "Angle Distribution", _angleHistograms, DEG2RAD(20), -limit, limit);
}

int main(int _argn, char **_argv)
{
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
		if (!Helper::getCloud(cloud, params))
			throw std::runtime_error("Can't load cloud");
		std::cout << "Loaded " << cloud->size() << " points in cloud\n";


		if (params.normalExecution)
		{
			pcl::PointNormal target = cloud->points[params.targetPoint];
			pcl::PointCloud<pcl::PointNormal>::Ptr patch;
			std::vector<BandPtr> bands = Calculator::calculateDescriptor(cloud, target, params, patch);

			// Calculate histograms
			std::cout << "Generating angle histograms\n";
			std::vector<Hist> histograms;
			Calculator::calculateAngleHistograms(bands, histograms, params.useProjection);

			// Write output
			std::cout << "Writing output\n";
			writeOuput(cloud, patch, bands, histograms, params);
		}
		else
		{
			pcl::PointNormal target = cloud->points[params.targetPoint];
			std::vector<BandPtr> bands = Calculator::calculateDescriptor(cloud, target, params);
		}
	}
	catch (std::exception &e)
	{
		std::cout << "ERROR: " << e.what() << std::endl;
	}

	std::cout << "Finished\n";
	return EXIT_SUCCESS;
}
