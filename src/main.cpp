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

void writeOuput(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const pcl::PointCloud<pcl::PointNormal>::Ptr &_patch, const std::vector<BandPtr> &_bands, const std::vector<Hist> &_angleHistograms, const ExecutionParams &_params, const int _targetIndex)
{
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr coloredCloud = Helper::createColorCloud(_cloud, Helper::getColor(0));

	std::cout << "Writing clouds to disk\n";
	pcl::io::savePCDFileASCII("./output/cloud.pcd", *coloredCloud);
	pcl::io::savePCDFileASCII("./output/patch.pcd", *Helper::createColorCloud(_patch, Helper::getColor(11)));

	(*coloredCloud)[_targetIndex].rgb = Helper::getColor(255, 0, 0);
	pcl::io::savePCDFileASCII("./output/pointPosition.pcd", *coloredCloud);

	std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> planes = Extractor::getBandPlanes(_bands, _params);

	std::ofstream sequences;
	sequences.open("./output/sequences", std::fstream::out);

	pcl::io::savePCDFileASCII("./output/plane.pcd", *Extractor::getTangentPlane(_cloud, _cloud->at(_targetIndex)));
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
	if (system("rm -rf ./output/*") != 0)
		std::cout << "ERROR, wrong command\n";

	std::cout << "Loading configuration file\n";
	if (!Config::load(CONFIG_LOCATION, _argn, _argv))
	{
		std::cout << "Can't read configuration file at " << CONFIG_LOCATION << "\n";
		return EXIT_FAILURE;
	}

	ExecutionParams params = Config::getExecutionParams();
	int index = params.targetPoint;
	std::cout << "Calcutating descriptor for point " << index << "\n";

	// Load cloud
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>());
	if (!Helper::getCloud(cloud, params))
	{
		std::cout << "ERROR: loading failed\n";
		return EXIT_FAILURE;
	}
	std::cout << "Loaded " << cloud->size() << " points in cloud\n";

	// Get target point and surface patch
	pcl::PointNormal point = cloud->points[index];
	pcl::PointCloud<pcl::PointNormal>::Ptr patch = Extractor::getNeighbors(cloud, point, params.patchSize);
	std::cout << "Patch size: " << patch->size() << "\n";

	// Extract bands
	std::vector<BandPtr> bands = Extractor::getBands(patch, point, params);
	if (!params.radialBands)
		Calculator::calculateSequences(bands, params, M_PI / 18, params.useProjection);

	// Calculate histograms
	std::cout << "Generating angle histograms\n";
	std::vector<Hist> histograms;
	Calculator::calculateAngleHistograms(bands, histograms, params.useProjection);

	// Write output
	std::cout << "Writing output\n";
	writeOuput(cloud, patch, bands, histograms, params, index);

	std::cout << "Finished\n";
	return EXIT_SUCCESS;
}
