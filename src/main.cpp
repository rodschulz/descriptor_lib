/**
 * Author: rodrigo
 * 2015
 */
#include <stdlib.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/rsd.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

using namespace std;
using namespace pcl;

void removeNANs(PointCloud<PointXYZ>::Ptr &_cloud)
{
	std::vector<int> mapping;
	removeNaNFromPointCloud(*_cloud, *_cloud, mapping);
}

float getColor(const float _r, const float _minR, const float _maxR)
{
	return ((_r - _minR) * 255) / _maxR;
}

int main(int argn, char **argv)
{
	system("rm -rf ./output/*");

	PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
	PointCloud<Normal>::Ptr normals(new PointCloud<Normal>);
	PointCloud<PrincipalRadiiRSD>::Ptr descriptors(new PointCloud<PrincipalRadiiRSD>());

	if (argn < 5)
	{
		cout << "Not enough arguments\n";
		return EXIT_FAILURE;
	}

	// Read a PCD file from disk.
	if (io::loadPCDFile<PointXYZ>(argv[1], *cloud) != 0)
		return EXIT_FAILURE;

	double searchRadius = atof(argv[2]);
	double planeRadius = atof(argv[3]);
	int targetPoint = atoi(argv[4]);

	// Remove NANs and downsample
	removeNANs(cloud);

	// Create a KD-Tree
	search::KdTree<PointXYZ>::Ptr tree(new search::KdTree<PointXYZ>);
	PointCloud<PointNormal> mls_points;
	MovingLeastSquares<PointXYZ, PointNormal> mls;
	mls.setComputeNormals(true);
	mls.setInputCloud(cloud);
	mls.setPolynomialFit(true);
	mls.setSearchMethod(tree);
	mls.setSearchRadius(0.03);
	// Reconstruct
	mls.process(mls_points);
	// Save output
	io::savePCDFile("./output/smoothed.pcd", mls_points);

	if (io::loadPCDFile<PointXYZ>("./output/smoothed.pcd", *cloud) != 0)
		return EXIT_FAILURE;

	cout << "Loaded " << cloud->points.size() << " points\n";

	cout << "Estimating normals\n";
	// Estimate normals
	NormalEstimation<PointXYZ, Normal> normalEstimation;
	normalEstimation.setInputCloud(cloud);
	normalEstimation.setRadiusSearch(0.03);
	search::KdTree<PointXYZ>::Ptr kdtree(new search::KdTree<PointXYZ>);
	normalEstimation.setSearchMethod(kdtree);
	normalEstimation.compute(*normals);

	cout << "Estimating RSD\n";

	// RSD estimation object.
	RSDEstimation<PointXYZ, Normal, PrincipalRadiiRSD> rsd;
	rsd.setInputCloud(cloud);
	rsd.setInputNormals(normals);
	rsd.setSearchMethod(kdtree);
	// Search radius, to look for neighbors.
	rsd.setRadiusSearch(searchRadius); //0.05
	// Plane radius. Any radius larger than this is considered infinite (a plane).
	rsd.setPlaneRadius(planeRadius);	//0.1
	// Do we want to save the full distance-angle histograms?
	rsd.setSaveHistograms(true);
	rsd.compute(*descriptors);

	float minRmin = numeric_limits<float>::max();
	float maxRmin = -numeric_limits<float>::max();
	float minRmax = numeric_limits<float>::max();
	float maxRmax = -numeric_limits<float>::max();
	for (int i = 0; i < descriptors->size(); i++)
	{
		PrincipalRadiiRSD radii = descriptors->points[i];

		minRmin = minRmin > radii.r_min ? radii.r_min : minRmin;
		maxRmin = maxRmin < radii.r_min ? radii.r_min : maxRmin;

		minRmax = minRmax > radii.r_max ? radii.r_max : minRmax;
		maxRmax = maxRmax < radii.r_max ? radii.r_max : maxRmax;
	}
	cout << "r_min: " << minRmin << " / " << maxRmin << "\n";
	cout << "r_max: " << minRmax << " / " << maxRmax << "\n";
	cout << "Extracting clouds\n";

	PointCloud<PointXYZRGB>::Ptr minRad(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr maxRad(new PointCloud<PointXYZRGB>);
	minRad->points.resize(cloud->width);
	minRad->height = 1;
	minRad->width = minRad->points.size();
	maxRad->points.resize(cloud->width);
	maxRad->height = 1;
	maxRad->width = maxRad->points.size();

	uint8_t r = 0, g = 0, b = 0;
	uint32_t target = ((uint32_t) 0 << 16 | (uint32_t) 255 << 8 | (uint32_t) 255);
	uint32_t rgb1, rgb2;
	for (int i = 0; i < cloud->width; i++)
	{
		PointXYZRGB p;
		p.x = cloud->points[i].x;
		p.y = cloud->points[i].y;
		p.z = cloud->points[i].z;

		// Adjust colors for min radius and max radius
		if (i == targetPoint)
		{
			rgb1 = target;
			rgb2 = target;

			cout << "Target point => r_min: " << descriptors->points[i].r_min << " / r_max: " << descriptors->points[i].r_max << "\n";
		}
		else
		{
			r = 150;
			g = getColor(descriptors->points[i].r_min, minRmin, maxRmin);
			b = 0;
			rgb1 = ((uint32_t) r << 16 | (uint32_t) g << 8 | (uint32_t) b);

			g = getColor(descriptors->points[i].r_max, minRmax, maxRmax);
			b = 0;
			rgb2 = ((uint32_t) r << 16 | (uint32_t) g << 8 | (uint32_t) b);
		}

		p.rgb = *reinterpret_cast<float*>(&rgb1);
		minRad->points[i] = p;
		p.rgb = *reinterpret_cast<float*>(&rgb2);
		maxRad->points[i] = p;
	}

	cout << "Writing clouds to disk\n";
	io::savePCDFileASCII("./output/input.pcd", *cloud);
	io::savePCDFileASCII("./output/normals.pcd", *normals);
	io::savePCDFileASCII("./output/minRad.pcd", *minRad);
	io::savePCDFileASCII("./output/maxRad.pcd", *maxRad);
	io::savePCDFileASCII("./output/descriptor.pcd", *descriptors);

	cout << "Finished\n";
	return EXIT_SUCCESS;
}
