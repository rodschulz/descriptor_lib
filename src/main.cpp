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
	// todo use a synthetic cloud to check if histograms are being built properly
	// todo remove curvature histogram generation

	system("rm -rf ./output/*");

	if (_argn < 3 || strcmp(_argv[1], "-h") == 0)
	{
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

//	CloudFactory::generateCube(0.5, Factory::makePointXYZ(1, 1, 1), cloud);
//	io::savePCDFileASCII("./output/cube.pcd", *cloud);
//	CloudFactory::generateCylinder(0.5, 1, Factory::makePointXYZ(1, 1, 1), cloud);
//	io::savePCDFileASCII("./output/cylinder.pcd", *cloud);
//	CloudFactory::generateSphere(1, Factory::makePointXYZ(1, 1, 1), cloud);
//	io::savePCDFileASCII("./output/sphere.pcd", *cloud);

	// Remove NANs and calculate normals
	Helper::removeNANs(cloud);
	PointCloud<Normal>::Ptr normals(new PointCloud<Normal>);
	Extractor::getNormals(cloud, params.normalEstimationRadius, normals);

	if (normals->empty())
	{
		cout << "ERROR: can't estimate normals\n";
		return EXIT_FAILURE;
	}

	PointXYZ point = cloud->points[index];
	Normal pointNormal = normals->points[index];

	PointCloud<PointXYZ>::Ptr surfacePatch(new PointCloud<PointXYZ>);
	PointCloud<Normal>::Ptr normalsPatch(new PointCloud<Normal>);
	Extractor::getNeighborsInRadius(cloud, normals, point, params.searchRadius, surfacePatch, normalsPatch);
	cout << "Patch size: " << surfacePatch->size() << "\n";

	Vector3f n1 = normals->at(9058).getNormalVector3fMap();
	Vector3f n2 = normals->at(2193).getNormalVector3fMap();
	double theta = Helper::angleBetween<Vector3f>(n1, n2);

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
	vector<double> curvatures;
	Helper::calculateMeanCurvature(bands, point, curvatures);
	vector<Hist> curvatureHistograms;
	cout << "Generating curvature histograms\n";
	Helper::calculateCurvatureHistograms(bands, point, curvatureHistograms);
	cout << "Generating angle histograms\n";
	vector<Hist> angleHistograms;
	Helper::calculateAngleHistograms(bands, point, angleHistograms);

	Writer::writeData("curvature.dat", curvatures, curvatureHistograms, angleHistograms);
//	Writer::writeHistogram("curvature", "Curvature Distribution", curvatureHistograms, 10);
	Writer::writeHistogram("angles", "Angle Distribution", angleHistograms, 18, 0, M_PI);

	// Write clouds to disk
	cout << "Writing clouds to disk\n";
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
