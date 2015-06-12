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

using namespace std;
using namespace pcl;

void writeOuput(const PointCloud<PointNormal>::Ptr &_cloud, const PointCloud<PointNormal>::Ptr &_patch, const vector<BandPtr> &_bands, const ExecutionParams &_params, const int _targetIndex)
{
	PointCloud<PointXYZRGBNormal>::Ptr coloredCloud = Helper::createColorCloud(_cloud, Helper::getColor(0));

	cout << "Writing clouds to disk\n";
	io::savePCDFileASCII("./output/cloud.pcd", *coloredCloud);
	io::savePCDFileASCII("./output/patch.pcd", *Helper::createColorCloud(_patch, Helper::getColor(11)));

	(*coloredCloud)[_targetIndex].rgb = Helper::getColor(255, 0, 0);
	io::savePCDFileASCII("./output/pointPosition.pcd", *coloredCloud);

	vector<PointCloud<PointNormal>::Ptr> planes = Extractor::getBandPlanes(_bands, _params);

	ofstream sequences;
	sequences.open("./output/sequences", fstream::out);

	io::savePCDFileASCII("./output/plane.pcd", *Extractor::getTangentPlane(_cloud, _cloud->at(_targetIndex)));
	for (size_t i = 0; i < _bands.size(); i++)
	{
		if (!_bands[i]->data->empty())
		{
			char name[100];
			sprintf(name, "./output/band%d.pcd", (int) i);
			io::savePCDFileASCII(name, *Helper::createColorCloud(_bands[i]->data, Helper::getColor(i + 1)));

			sequences << "band " << i << "\n";
			sequences << "\tmean  : " << _bands[i]->sequenceMean << "\n";
			sequences << "\tmedian: " << _bands[i]->sequenceMedian << "\n";

			sprintf(name, "./output/planeBand%d.pcd", (int) i);
			io::savePCDFileASCII(name, *planes[i]);
		}
	}

	sequences.close();
}

int main(int _argn, char **_argv)
{
	if (system("rm -rf ./output/*") != 0)
		cout << "ERROR, wrong command\n";

	cout << "Loading configuration file\n";
	if (!Config::load(CONFIG_LOCATION, _argn, _argv))
	{
		cout << "Can't read configuration file at " << CONFIG_LOCATION << "\n";
		return EXIT_FAILURE;
	}

	ExecutionParams params = Config::getExecutionParams();
	int index = params.targetPoint;
	cout << "Calcutating descriptor for point " << index << "\n";

	// Load cloud
	PointCloud<PointNormal>::Ptr cloud(new PointCloud<PointNormal>());
	if (!Helper::getCloud(cloud, params))
	{
		cout << "ERROR: loading failed\n";
		return EXIT_FAILURE;
	}
	cout << "Loaded " << cloud->size() << " points in cloud\n";

	// Get target point and surface patch
	PointNormal point = cloud->points[index];
	PointCloud<PointNormal>::Ptr patch = Extractor::getNeighbors(cloud, point, params.patchSize);
	cout << "Patch size: " << patch->size() << "\n";

	// Extract bands
	vector<BandPtr> bands = Extractor::getBands(patch, point, params);
	if (!params.radialBands)
		Calculator::calculateSequences(bands, params.sequenceBin, M_PI / 18, params.useProjection);

	// Calculate histograms
	cout << "Generating angle histograms\n";
	vector<Hist> angleHistograms;
	Calculator::calculateAngleHistograms(bands, angleHistograms, params.useProjection);
	Writer::writeHistogram("angles", "Angle Distribution", angleHistograms, 18, 0, M_PI);

	// Write
	writeOuput(cloud, patch, bands, params, index);

	cout << "Finished\n";
	return EXIT_SUCCESS;
}
