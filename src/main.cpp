/**
 * Author: rodrigo
 * 2015
 */
#include <stdlib.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <string>
#include "Helper.h"
#include "Factory.h"
#include "Extractor.h"
#include "Parser.h"
#include "Hist.h"
#include "Writer.h"
#include "CloudFactory.h"

using namespace std;
using namespace pcl;

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

	// Get target point
	PointNormal point = cloud->points[index];

	// Extract patch
	PointCloud<PointNormal>::Ptr patch(new PointCloud<PointNormal>());
	Extractor::getNeighborsInRadius(cloud, point, params.searchRadius, patch);
	cout << "Patch size: " << patch->size() << "\n";

	// Extract interesting bands
	vector<Band> bands;
	double step = Extractor::getBands(patch, point, params, bands);

	// Calculate histograms
	cout << "Generating angle histograms\n";
	vector<Hist> angleHistograms;
	Helper::calculateAngleHistograms(bands, point, angleHistograms);
	Writer::writeHistogram("angles", "Angle Distribution", angleHistograms, 18, 0, M_PI);

	////////////

	// Write clouds to disk
	cout << "Writing clouds to disk\n";
	io::savePCDFileASCII("./output/cloud.pcd", *cloud);
	io::savePCDFileASCII("./output/patch.pcd", *patch);

	PointCloud<PointXYZRGB>::Ptr coloredCloud = Helper::createColorCloud(cloud, 255, 0, 0);
	(*coloredCloud)[index].rgb = Helper::getColor(0, 255, 0);
	io::savePCDFileASCII("./output/pointPosition.pcd", *coloredCloud);

	io::savePCDFileASCII("./output/plane.pcd", *Extractor::getTangentPlane(cloud, point));
	for (size_t i = 0; i < bands.size(); i++)
	{
		if (!bands[i].data->empty())
		{
			char name[100];
			sprintf(name, "./output/band%d.pcd", (int) i);
			io::savePCDFileASCII(name, *bands[i].data);
		}
	}

	cout << "Finished\n";
	return EXIT_SUCCESS;
}
