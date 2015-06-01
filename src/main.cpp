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

int main(int _argn, char **_argv)
{
	// todo remove curvature histogram generation

	system("rm -rf ./output/*");

	if (_argn < 3 || strcmp(_argv[1], "-h") == 0)
	{
		Parser::printUsage();
		return EXIT_FAILURE;
	}

	ExecutionParams params = Parser::parseExecutionParams(_argn, _argv);
	int index = params.targetPoint;

	PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>());
	if (!getCloud(cloud, params))
	{
		cout << "ERROR: Can't get cloud\n";
		return EXIT_FAILURE;
	}
	cout << "Cloud loaded points: " << cloud->size() << "\n";

	// Remove NANs and calculate normals
	Helper::removeNANs(cloud);
	PointCloud<Normal>::Ptr normals(new PointCloud<Normal>());
	Extractor::getNormals(cloud, params.normalEstimationRadius, normals);

	if (normals->empty())
	{
		cout << "ERROR: can't estimate normals\n";
		return EXIT_FAILURE;
	}

	// Create a joint cloud of points and normals
	PointCloud<PointNormal>::Ptr jointCloud(new PointCloud<PointNormal>());
	concatenateFields(*cloud, *normals, *jointCloud);

	// Get target point and normal
	PointXYZ point = cloud->points[index];
	Normal pointNormal = normals->points[index];

	// Extract patch
	PointCloud<PointXYZ>::Ptr surfacePatch(new PointCloud<PointXYZ>);
	PointCloud<Normal>::Ptr normalsPatch(new PointCloud<Normal>);
	Extractor::getNeighborsInRadius(cloud, normals, point, params.searchRadius, surfacePatch, normalsPatch);
	cout << "Patch size: " << surfacePatch->size() << "\n";

	// Create a colored version of the could to check the target point's position
	PointCloud<PointXYZRGB>::Ptr coloredCloud(new PointCloud<PointXYZRGB>());
	Helper::createColorCloud(cloud, coloredCloud, 255, 0, 0);
	(*coloredCloud)[index].rgb = Helper::getColor(0, 255, 0);

	// Create a cloud holding the tangent plane
	PointCloud<PointXYZRGB>::Ptr tangentPlane(new PointCloud<PointXYZRGB>());
	Extractor::getTangentPlane(cloud, point, pointNormal, tangentPlane);

	// Extract interesting bands
	vector<Band> bands;
	double step = Extractor::getBands(surfacePatch, normalsPatch, point, pointNormal, params, bands);

	// Calculate histograms
	cout << "Generating angle histograms\n";
	vector<Hist> angleHistograms;
	Helper::calculateAngleHistograms(bands, point, angleHistograms);
	Writer::writeHistogram("angles", "Angle Distribution", angleHistograms, 18, 0, M_PI);

	// Write clouds to disk
	cout << "Writing clouds to disk\n";
	io::savePCDFileASCII("./output/jointCloud.pcd", *jointCloud);
	io::savePCDFileASCII("./output/normals.pcd", *normals);
	io::savePCDFileASCII("./output/patch.pcd", *surfacePatch);
	io::savePCDFileASCII("./output/pointPosition.pcd", *coloredCloud);
	io::savePCDFileASCII("./output/plane.pcd", *tangentPlane);
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
