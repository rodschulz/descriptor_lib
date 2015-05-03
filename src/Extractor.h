/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;
using namespace pcl;

struct Band
{
	PointCloud<PointXYZ>::Ptr dataBand;
	PointCloud<Normal>::Ptr normalBand;

	Band()
	{
		dataBand = PointCloud<PointXYZ>::Ptr(new PointCloud<PointXYZ>());
		normalBand = PointCloud<Normal>::Ptr(new PointCloud<Normal>());
	}
};

class Extractor
{
public:
	static void getNeighborsInRadius(const PointXYZ &_searchPoint, const double _searchRadius, const PointCloud<PointXYZ>::Ptr &_cloud, PointCloud<PointXYZ>::Ptr &_outputNeighbors);
	static void getNormals(const PointCloud<PointXYZ>::Ptr &_cloud, const double _searchRadius, PointCloud<Normal>::Ptr &_normals);
	static void getBands(const PointCloud<PointXYZ>::Ptr &_cloud, const PointCloud<Normal>::Ptr &_normals, const int &_targetPoint, const double _bandWidth, vector<Band> &_bands);
	static void getTangentPlane(const PointCloud<PointXYZ>::Ptr &_cloud, const PointXYZ &_point, const Normal &_pointNormal, PointCloud<PointXYZRGB>::Ptr &_tangentPlane);

private:
	Extractor();
	~Extractor();
};
