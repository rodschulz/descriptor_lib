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

#include <pcl/visualization/pcl_visualizer.h>

using namespace std;
using namespace pcl;

bool getCloud(PointCloud<PointXYZ>::Ptr &_cloud, const ExecutionParams &_params)
{
	if (!_params.useSynthetic)
	{
		// Read a PCD file from disk
		if (io::loadPCDFile<PointXYZ>(_params.inputLocation, *_cloud) != 0)
		{
			cout << "Can't read cloud from file " << _params.inputLocation << "\n";
			return false;
		}
	}
	else
	{
		switch (_params.synCloudType)
		{
			case CUBE:
				CloudFactory::generateCube(0.3, Factory::makePointXYZ(0.3, 0.3, 0.3), _cloud);
				break;

			case CYLINDER:
				CloudFactory::generateCylinder(0.2, 0.5, Factory::makePointXYZ(0.2, 0.5, 0.2), _cloud);
				break;

			case SPHERE:
				CloudFactory::generateSphere(0.2, Factory::makePointXYZ(0.2, 0.2, 0.2), _cloud);
				break;

			default:
				_cloud->clear();
		}
	}

	return true;
}

bool getFullCloud(PointCloud<PointXYZ>::Ptr &_cloudXYZ, const ExecutionParams &_params, PointCloud<PointNormal>::Ptr &_fullCloud)
{
	// Remove NANs and calculate normals
	Helper::removeNANs(_cloudXYZ);
	PointCloud<Normal>::Ptr normals(new PointCloud<Normal>());
	Extractor::getNormals(_cloudXYZ, _params.normalEstimationRadius, normals);

	if (normals->empty())
	{
		cout << "ERROR: can't estimate normals\n";
		return false;
	}

	// Create a XYZ data and normals
	_fullCloud->clear();
	concatenateFields(*_cloudXYZ, *normals, *_fullCloud);

	return true;
}

int main(int _argn, char **_argv)
{
	system("rm -rf ./output/*");

	if (_argn < 3 || strcmp(_argv[1], "-h") == 0)
	{
		Parser::printUsage();
		return EXIT_FAILURE;
	}

	ExecutionParams params = Parser::parseExecutionParams(_argn, _argv);
	int index = params.targetPoint;

	// Load XYZ data
	PointCloud<PointXYZ>::Ptr cloudXYZ(new PointCloud<PointXYZ>());
	if (!getCloud(cloudXYZ, params))
	{
		cout << "ERROR: Can't get cloud\n";
		return EXIT_FAILURE;
	}
	cout << "Cloud loaded points: " << cloudXYZ->size() << "\n";

	// Calculate normals and join them to get the full cloud
	PointCloud<PointNormal>::Ptr cloud(new PointCloud<PointNormal>());
	if (!getFullCloud(cloudXYZ, params, cloud))
		return EXIT_FAILURE;

	// Get target point
	PointNormal point = cloud->points[index];

	// Extract patch
	PointCloud<PointNormal>::Ptr patch(new PointCloud<PointNormal>());
	Extractor::getNeighborsInRadius(cloud, point, params.searchRadius, patch);
	cout << "Patch size: " << patch->size() << "\n";

	// Create a colored version of the could to check the target point's position
	PointCloud<PointXYZRGB>::Ptr coloredCloud(new PointCloud<PointXYZRGB>());
	Helper::createColorCloud(cloudXYZ, coloredCloud, 255, 0, 0);
	(*coloredCloud)[index].rgb = Helper::getColor(0, 255, 0);

	// Extract interesting bands
	vector<Band> bands;
	double step = Extractor::getBands(patch, point, params, bands);

	// Calculate histograms
	cout << "Generating angle histograms\n";
	vector<Hist> angleHistograms;
	Helper::calculateAngleHistograms(bands, point, angleHistograms);
	Writer::writeHistogram("angles", "Angle Distribution", angleHistograms, 18, 0, M_PI);

	// Write clouds to disk
	cout << "Writing clouds to disk\n";
	io::savePCDFileASCII("./output/cloud.pcd", *cloud);
	io::savePCDFileASCII("./output/patch.pcd", *patch);
	io::savePCDFileASCII("./output/pointPosition.pcd", *coloredCloud);
	io::savePCDFileASCII("./output/plane.pcd", *Extractor::getTangentPlane(cloudXYZ, point));
	for (size_t i = 0; i < bands.size(); i++)
	{
		if (!bands[i].dataBand->empty())
		{
			char name[100];
			sprintf(name, "./output/band%d.pcd", (int) i);
			io::savePCDFileASCII(name, *bands[i].dataBand);
		}
	}

	cout << "Finished\n";
	return EXIT_SUCCESS;
}
