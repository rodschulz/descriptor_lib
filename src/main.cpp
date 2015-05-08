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

using namespace std;
using namespace pcl;

int main(int _argn, char **_argv)
{
	// todo create radial bands to calculate the descriptor in different surroundings
	// todo implement generation of histograms of curvature in each band
	// todo implement directions in bands
	// todo implement dynamic number of bands

	system("rm -rf ./output/*");

	// Show help
	if (strcmp(_argv[1], "-h") == 0)
	{
		Parser::printUsage();
		return EXIT_SUCCESS;
	}

	if (_argn < 3)
	{
		cout << "Not enough arguments\n";
		Parser::printUsage();
		return EXIT_FAILURE;
	}

	ExecutionParams params = Parser::parseExecutionParams(_argn, _argv);
	int index = params.targetPoint;
	int method = params.method;

	// Read a PCD file from disk.
	PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
	if (io::loadPCDFile<PointXYZ>(params.inputLocation, *cloud) != 0)
	{
		cout << "ERROR: Can't read file from disk (" << _argv[1] << ")\n";
		return EXIT_FAILURE;
	}
	cout << "Cloud loaded points: " << cloud->size() << "\n";

	// Remove NANs and calculate normals
	Helper::removeNANs(cloud);
	PointCloud<Normal>::Ptr normals(new PointCloud<Normal>);
	Extractor::getNormals(cloud, 0.01, normals);

	PointXYZ point = cloud->points[index];
	Normal pointNormal = normals->points[index];

	PointCloud<PointXYZ>::Ptr surfacePatch(new PointCloud<PointXYZ>);
	PointCloud<Normal>::Ptr normalsPatch(new PointCloud<Normal>);
	if (method == 0)
	{
		Extractor::getNeighborsInRadius(cloud, normals, point, params.searchRadius, surfacePatch, normalsPatch);
	}
	else if (method == 1)
	{
		//getNeighborsK(searchPoint, params.neighborsNumber, cloud, neighbors);
	}

	// Create a colored version of the could to check the target point's position
	PointCloud<PointXYZRGB>::Ptr coloredCloud(new PointCloud<PointXYZRGB>());
	Helper::createColorCloud(cloud, coloredCloud, 255, 0, 0);
	(*coloredCloud)[index].rgb = Helper::getColor(0, 255, 0);

	// Create a cloud holding the tangent plane
	PointCloud<PointXYZRGB>::Ptr tangentPlane(new PointCloud<PointXYZRGB>());
	Extractor::getTangentPlane(cloud, point, pointNormal, tangentPlane);

	// Extract interesting bands
	vector<Band> bands;
	double bandAngularStep = Extractor::getBands(surfacePatch, normalsPatch, point, pointNormal, params, bands);
	for (size_t i = 0; i < bands.size(); i++)
	{
		char name[100];
		sprintf(name, "./output/band%d.pcd", (int) i);
		io::savePCDFileASCII(name, *bands[i].dataBand);
	}

	// Calculate mean curvature
	vector<double> curvatures;
	Helper::calculateMeanCurvature(bands, point, curvatures);
	for (size_t i = 0; i < curvatures.size(); i++)
		cout << "Curvature " << bandAngularStep * i << ": " << curvatures[i] << "\n";

	// Write clouds to disk
	cout << "Writing clouds to disk\n";
	io::savePCDFileASCII("./output/normals.pcd", *normals);
	io::savePCDFileASCII("./output/patch.pcd", *surfacePatch);
	io::savePCDFileASCII("./output/pointPosition.pcd", *coloredCloud);
	io::savePCDFileASCII("./output/plane.pcd", *tangentPlane);

	cout << "Finished\n";
	return EXIT_SUCCESS;
}
