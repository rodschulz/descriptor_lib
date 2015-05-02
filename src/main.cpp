/**
 * Author: rodrigo
 * 2015
 */
#include <stdlib.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/features/normal_3d.h>
#include "Helper.h"
#include "Factory.h"

using namespace std;
using namespace pcl;

void getNeighborsInRadius(const PointXYZ &_searchPoint, const double _searchRadius, const PointCloud<PointXYZ>::Ptr &_cloud, PointCloud<PointXYZ>::Ptr &_outputNeighbors)
{
	KdTreeFLANN<PointXYZ> kdtree;
	kdtree.setInputCloud(_cloud);

	vector<int> pointIndices;
	vector<float> pointRadiusSquaredDistance;
	kdtree.radiusSearch(_searchPoint, _searchRadius, pointIndices, pointRadiusSquaredDistance);

	_outputNeighbors->points.clear();
	_outputNeighbors->points.reserve(pointIndices.size());
	_outputNeighbors->width = pointIndices.size();
	_outputNeighbors->height = 1;

	for (size_t i = 0; i < pointIndices.size(); i++)
		_outputNeighbors->points.push_back(_cloud->points[pointIndices[i]]);
}

void getDataBands(const PointXYZ &_targetPoint, const double _bandWidth, const PointCloud<PointXYZ>::Ptr &_cloud, vector<PointCloud<PointXYZ>::Ptr> &_bands, vector<PointCloud<PointXYZRGB>::Ptr> &_limits)
{
	_bands.clear();
	_bands.push_back(PointCloud<PointXYZ>::Ptr(new PointCloud<PointXYZ>()));
	_bands.push_back(PointCloud<PointXYZ>::Ptr(new PointCloud<PointXYZ>()));
	_bands.push_back(PointCloud<PointXYZ>::Ptr(new PointCloud<PointXYZ>()));
	_bands.push_back(PointCloud<PointXYZ>::Ptr(new PointCloud<PointXYZ>()));

	_limits.clear();
	_limits.push_back(PointCloud<PointXYZRGB>::Ptr(new PointCloud<PointXYZRGB>()));
	_limits.push_back(PointCloud<PointXYZRGB>::Ptr(new PointCloud<PointXYZRGB>()));
	_limits.push_back(PointCloud<PointXYZRGB>::Ptr(new PointCloud<PointXYZRGB>()));
	_limits.push_back(PointCloud<PointXYZRGB>::Ptr(new PointCloud<PointXYZRGB>()));

	double targetX = _targetPoint.x;
	double targetY = _targetPoint.y;
	double halfBand = _bandWidth / 2;

	double m1 = 1;
	double m3 = -1;
	double constant = halfBand * sqrt(2);

	for (int i = 0; i < _cloud->width; i++)
	{
		double x = _cloud->points[i].x;
		double y = _cloud->points[i].y;
		double z = _cloud->points[i].z;

		// point is in the horizontal band
		if (abs(y - targetY) < halfBand)
			_bands[0]->push_back(_cloud->points[i]);

		// point is in the 45 degree band
		double upperLimit1 = m1 * (x - targetX) + targetY + constant;
		double lowerLimit1 = m1 * (x - targetX) + targetY - constant;
		if (lowerLimit1 <= y && y <= upperLimit1)
			_bands[1]->push_back(_cloud->points[i]);

		// point is in the vertical band
		if (abs(x - targetX) < halfBand)
			_bands[2]->push_back(_cloud->points[i]);

		// point is in the 45 degree band
		double upperLimit3 = m3 * (x - targetX) + targetY + constant;
		double lowerLimit3 = m3 * (x - targetX) + targetY - constant;
		if (lowerLimit3 <= y && y <= upperLimit3)
			_bands[3]->push_back(_cloud->points[i]);

		_limits[0]->push_back(Factory::makePointXYZRGB(x, targetY + halfBand, z, 255, 0, 0));
		_limits[0]->push_back(Factory::makePointXYZRGB(x, targetY - halfBand, z, 255, 175, 0));

		_limits[1]->push_back(Factory::makePointXYZRGB(x, upperLimit1, z, 175, 255, 255));
		_limits[1]->push_back(Factory::makePointXYZRGB(x, lowerLimit1, z, 0, 255, 255));

		_limits[2]->push_back(Factory::makePointXYZRGB(targetX + halfBand, y, z, 0, 255, 0));
		_limits[2]->push_back(Factory::makePointXYZRGB(targetX - halfBand, y, z, 0, 255, 175));

		_limits[3]->push_back(Factory::makePointXYZRGB(x, upperLimit3, z, 255, 255, 175));
		_limits[3]->push_back(Factory::makePointXYZRGB(x, lowerLimit3, z, 255, 255, 0));
	}

	_bands[0]->height = _bands[1]->height = _bands[2]->height = _bands[3]->height = 1;
	_bands[0]->width = _bands[0]->points.size();
	_bands[1]->width = _bands[1]->points.size();
	_bands[2]->width = _bands[2]->points.size();
	_bands[3]->width = _bands[3]->points.size();
}

int main(int _argn, char **_argv)
{
	system("rm -rf ./output/*");

	if (_argn < 5)
	{
		cout << "Not enough arguments\n";
		return EXIT_FAILURE;
	}

	// Read a PCD file from disk.
	PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
	if (io::loadPCDFile<PointXYZ>(_argv[1], *cloud) != 0)
	{
		cout << "ERROR: Can't read file from disk (" << _argv[1] << ")\n";
		return EXIT_FAILURE;
	}

	int targetPoint = atoi(_argv[2]);
	int method = atoi(_argv[3]);

	Helper::removeNANs(cloud);

	PointXYZ searchPoint = cloud->points[targetPoint];
	PointCloud<PointXYZ>::Ptr patch(new PointCloud<PointXYZ>);
	if (method == 0)
	{
		double searchRadius = atof(_argv[4]);
		getNeighborsInRadius(searchPoint, searchRadius, cloud, patch);
	}
	else if (method == 1)
	{
		int neighborsNumber = atoi(_argv[4]);
		//getNeighborsK(searchPoint, neighborsNumber, cloud, neighbors);
	}

	double bandSize = 0.01;
	vector<PointCloud<PointXYZ>::Ptr> bands;
	vector<PointCloud<PointXYZRGB>::Ptr> limits;
	getDataBands(searchPoint, bandSize, patch, bands, limits);

	PointCloud<PointXYZRGB>::Ptr coloredCloud(new PointCloud<PointXYZRGB>());
	Helper::createColorCloud(cloud, coloredCloud, 255, 0, 0);
	coloredCloud->points[targetPoint].rgb = Helper::getColor(0, 255, 0);

	cout << "Writing clouds to disk\n";
	io::savePCDFileASCII("./output/patch.pcd", *patch);
	io::savePCDFileASCII("./output/pointPosition.pcd", *coloredCloud);
	io::savePCDFileASCII("./output/limits0.pcd", *limits[0]);
	io::savePCDFileASCII("./output/limits45.pcd", *limits[1]);
	io::savePCDFileASCII("./output/limits90.pcd", *limits[2]);
	io::savePCDFileASCII("./output/limits135.pcd", *limits[3]);
	io::savePCDFileASCII("./output/band0.pcd", *bands[0]);
	io::savePCDFileASCII("./output/band2.pcd", *bands[2]);
	io::savePCDFileASCII("./output/band1.pcd", *bands[1]);
	io::savePCDFileASCII("./output/band3.pcd", *bands[3]);

// Estimate normals
//	NormalEstimation<PointXYZ, Normal> normalEstimation;
//	normalEstimation.setInputCloud(cloud);
//	normalEstimation.setRadiusSearch(0.03);
//	search::KdTree<PointXYZ>::Ptr kdtree(new search::KdTree<PointXYZ>);
//	normalEstimation.setSearchMethod(kdtree);
//	PointCloud<Normal>::Ptr normals(new PointCloud<Normal>);
//	normalEstimation.compute(*normals);

//	Eigen::Vector4f p = normals->points[i].getNormalVector4fMap ();
//	Vector4f q = normals->points[j].getNormalVector4fMap ();
//	Vector4f r = p.dot(q);

//	PrincipalCurvaturesEstimation<PointXYZ, Normal, PointXYZ>::computePointPrincipalCurvatures();

//	PointCloud<Normal>::Ptr normals(new PointCloud<Normal>);
//	PointCloud<PrincipalRadiiRSD>::Ptr descriptors(new PointCloud<PrincipalRadiiRSD>());

//	// Create a KD-Tree
//	search::KdTree<PointXYZ>::Ptr tree(new search::KdTree<PointXYZ>);
//	PointCloud<PointNormal> mls_points;
//	MovingLeastSquares<PointXYZ, PointNormal> mls;
//	mls.setComputeNormals(true);
//	mls.setInputCloud(cloud);
//	mls.setPolynomialFit(true);
//	mls.setSearchMethod(tree);
//	mls.setSearchRadius(0.03);
//	// Reconstruct
//	mls.process(mls_points);
//	// Save output
//	io::savePCDFile("./output/smoothed.pcd", mls_points);
//
//	if (io::loadPCDFile<PointXYZ>("./output/smoothed.pcd", *cloud) != 0)
//		return EXIT_FAILURE;
//
//	cout << "Loaded " << cloud->points.size() << " points\n";
//
//	cout << "Estimating normals\n";
//	// Estimate normals
//	NormalEstimation<PointXYZ, Normal> normalEstimation;
//	normalEstimation.setInputCloud(cloud);
//	normalEstimation.setRadiusSearch(0.03);
//	search::KdTree<PointXYZ>::Ptr kdtree(new search::KdTree<PointXYZ>);
//	normalEstimation.setSearchMethod(kdtree);
//	normalEstimation.compute(*normals);
//
//	cout << "Estimating RSD\n";
//
//	// RSD estimation object.
//	RSDEstimation<PointXYZ, Normal, PrincipalRadiiRSD> rsd;
//	rsd.setInputCloud(cloud);
//	rsd.setInputNormals(normals);
//	rsd.setSearchMethod(kdtree);
//	// Search radius, to look for neighbors.
//	rsd.setRadiusSearch(searchRadius); //0.05
//	// Plane radius. Any radius larger than this is considered infinite (a plane).
//	rsd.setPlaneRadius(planeRadius);	//0.1
//	// Do we want to save the full distance-angle histograms?
//	rsd.setSaveHistograms(true);
//	rsd.compute(*descriptors);
//
//	float minRmin = numeric_limits<float>::max();
//	float maxRmin = -numeric_limits<float>::max();
//	float minRmax = numeric_limits<float>::max();
//	float maxRmax = -numeric_limits<float>::max();
//	for (int i = 0; i < descriptors->size(); i++)
//	{
//		PrincipalRadiiRSD radii = descriptors->points[i];
//
//		minRmin = minRmin > radii.r_min ? radii.r_min : minRmin;
//		maxRmin = maxRmin < radii.r_min ? radii.r_min : maxRmin;
//
//		minRmax = minRmax > radii.r_max ? radii.r_max : minRmax;
//		maxRmax = maxRmax < radii.r_max ? radii.r_max : maxRmax;
//	}
//	cout << "r_min: " << minRmin << " / " << maxRmin << "\n";
//	cout << "r_max: " << minRmax << " / " << maxRmax << "\n";
//	cout << "Extracting clouds\n";
//
//	PointCloud<PointXYZRGB>::Ptr minRad(new PointCloud<PointXYZRGB>);
//	PointCloud<PointXYZRGB>::Ptr maxRad(new PointCloud<PointXYZRGB>);
//	minRad->points.resize(cloud->width);
//	minRad->height = 1;
//	minRad->width = minRad->points.size();
//	maxRad->points.resize(cloud->width);
//	maxRad->height = 1;
//	maxRad->width = maxRad->points.size();
//
//	uint8_t r = 0, g = 0, b = 0;
//	uint32_t target = ((uint32_t) 0 << 16 | (uint32_t) 255 << 8 | (uint32_t) 255);
//	uint32_t rgb1, rgb2;
//	for (int i = 0; i < cloud->width; i++)
//	{
//		PointXYZRGB p;
//		p.x = cloud->points[i].x;
//		p.y = cloud->points[i].y;
//		p.z = cloud->points[i].z;
//
//		// Adjust colors for min radius and max radius
//		if (i == targetPoint)
//		{
//			rgb1 = target;
//			rgb2 = target;
//
//			cout << "Target point => r_min: " << descriptors->points[i].r_min << " / r_max: " << descriptors->points[i].r_max << "\n";
//		}
//		else
//		{
//			r = 150;
//			g = getColor(descriptors->points[i].r_min, minRmin, maxRmin);
//			b = 0;
//			rgb1 = ((uint32_t) r << 16 | (uint32_t) g << 8 | (uint32_t) b);
//
//			g = getColor(descriptors->points[i].r_max, minRmax, maxRmax);
//			b = 0;
//			rgb2 = ((uint32_t) r << 16 | (uint32_t) g << 8 | (uint32_t) b);
//		}
//
//		p.rgb = *reinterpret_cast<float*>(&rgb1);
//		minRad->points[i] = p;
//		p.rgb = *reinterpret_cast<float*>(&rgb2);
//		maxRad->points[i] = p;
//	}

//	cout << "Writing clouds to disk\n";
//	io::savePCDFileASCII("./output/input.pcd", *cloud);
//	io::savePCDFileASCII("./output/normals.pcd", *normals);
//	io::savePCDFileASCII("./output/minRad.pcd", *minRad);
//	io::savePCDFileASCII("./output/maxRad.pcd", *maxRad);
//	io::savePCDFileASCII("./output/descriptor.pcd", *descriptors);

	cout << "Finished\n";
	return EXIT_SUCCESS;
}
