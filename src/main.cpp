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

using namespace std;
using namespace pcl;

int main(int _argn, char **_argv)
{
	// todo implement generation of histograms of curvature in each band

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
	Extractor::getNeighborsInRadius(cloud, normals, point, params.searchRadius, surfacePatch, normalsPatch);

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

	// Calculate mean curvature
	vector<double> curvatures;
	Helper::calculateMeanCurvature(bands, point, curvatures);
	for (size_t i = 0; i < curvatures.size(); i++)
		cout << "Curvature band " << i << ": " << curvatures[i] << "\n";
	
	cout << "***** Curvatures *****\n";
	vector <Hist> curvatureHistograms;
	Helper::calculateCurvatureHistograms(bands, point, curvatureHistograms);
	for (size_t i = 0; i < curvatureHistograms.size(); i++)
	{
		Bins bins;
		curvatureHistograms[i].getBins(8, bins);
		cout << "BAND" << i << ": ";
		printBins(bins);
	}
	
	cout << "***** Angles *****\n";
	vector <Hist> angleHistograms;
	Helper::calculateAngleHistograms(bands, point, angleHistograms);
	for (size_t i = 0; i < angleHistograms.size(); i++)
	{
		Bins bins;
		angleHistograms[i].getBins(8, 0, M_PI, bins);
		cout << "BAND" << i << ": ";
		printBins(bins);
	}

	// Write clouds to disk
	cout << "Writing clouds to disk\n";
	io::savePCDFileASCII("./output/normals.pcd", *normals);
	io::savePCDFileASCII("./output/patch.pcd", *surfacePatch);
	io::savePCDFileASCII("./output/pointPosition.pcd", *coloredCloud);
	io::savePCDFileASCII("./output/plane.pcd", *tangentPlane);
	for (size_t i = 0; i < bands.size(); i++)
	{
		char name[100];
		sprintf(name, "./output/band%d.pcd", (int) i);
		io::savePCDFileASCII(name, *bands[i].dataBand);
	}

	cout << "Finished\n";
	return EXIT_SUCCESS;
}
