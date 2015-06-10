/**
 * Author: rodrigo
 * 2015
 */
#include <stdlib.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <string>
#include "Helper.h"
#include "Extractor.h"
#include "Parser.h"
#include "Hist.h"
#include "Writer.h"
#include "Calculator.h"

using namespace std;
using namespace pcl;

void writeOuput(const PointCloud<PointNormal>::Ptr &_cloud, const PointCloud<PointNormal>::Ptr &_patch, const vector<BandPtr> &_bands, const int _targetIndex)
{
	cout << "Writing clouds to disk\n";
	io::savePCDFileASCII("./output/cloud.pcd", *_cloud);
	io::savePCDFileASCII("./output/patch.pcd", *_patch);

	PointCloud<PointXYZRGB>::Ptr coloredCloud = Helper::createColorCloud(_cloud, 255, 0, 0);
	(*coloredCloud)[_targetIndex].rgb = Helper::getColor(0, 255, 0);
	io::savePCDFileASCII("./output/pointPosition.pcd", *coloredCloud);

	io::savePCDFileASCII("./output/plane.pcd", *Extractor::getTangentPlane(_cloud, _cloud->at(_targetIndex)));
	for (size_t i = 0; i < _bands.size(); i++)
	{
		if (!_bands[i]->data->empty())
		{
			char name[100];
			sprintf(name, "./output/band%d.pcd", (int) i);
			io::savePCDFileASCII(name, *_bands[i]->data);
		}
	}
}

int main(int _argn, char **_argv)
{
	if (system("rm -rf ./output/*") != 0)
		cout << "ERROR, wrong command\n";

	if (_argn < 3 || strcmp(_argv[1], "-h") == 0)
	{
		Parser::printUsage();
		return EXIT_FAILURE;
	}

	ExecutionParams params = Parser::parseExecutionParams(_argn, _argv);
	int index = params.targetPoint;

	// Load cloud
	PointCloud<PointNormal>::Ptr cloud(new PointCloud<PointNormal>());
	if (!Helper::getCloudAndNormals(cloud, params))
	{
		cout << "ERROR: loading failed\n";
		return EXIT_FAILURE;
	}
	cout << "Loaded " << cloud->size() << " points in cloud\n";

	// Get target point and surface patch
	PointNormal point = cloud->points[index];
	PointCloud<PointNormal>::Ptr patch = Extractor::getNeighbors(cloud, point, params.searchRadius);
	cout << "Patch size: " << patch->size() << "\n";

	// Extract bands
	vector<BandPtr> bands = Extractor::getBands(patch, point, params);

	if (!params.radialBands)
		Calculator::getSequences(bands, params.bandWidth);

	// Calculate histograms
	cout << "Generating angle histograms\n";
	vector<Hist> angleHistograms;
	Calculator::calculateAngleHistograms(bands, angleHistograms);
	Writer::writeHistogram("angles", "Angle Distribution", angleHistograms, 18, 0, M_PI);

	// Write
	writeOuput(cloud, patch, bands, index);

	cout << "Finished\n";
	return EXIT_SUCCESS;
}
