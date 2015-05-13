/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <eigen3/Eigen/src/Geometry/Hyperplane.h>
#include "Parser.h"

using namespace std;
using namespace pcl;
using namespace Eigen;

struct Band
{
	PointCloud<PointXYZ>::Ptr dataBand;
	PointCloud<Normal>::Ptr normalBand;
	Vector3f pointNormal;
	Hyperplane<float, 3> planeAlong;
	bool radialBand;

	Band(const Vector3f &_pointNormal, const Hyperplane<float, 3> &_planeAlong)
	{
		dataBand = PointCloud<PointXYZ>::Ptr(new PointCloud<PointXYZ>());
		normalBand = PointCloud<Normal>::Ptr(new PointCloud<Normal>());
		pointNormal = _pointNormal;
		planeAlong = _planeAlong;
		radialBand = false;
	}

	Band(const Vector3f &_pointNormal, const bool &_radialBand)
	{
		dataBand = PointCloud<PointXYZ>::Ptr(new PointCloud<PointXYZ>());
		normalBand = PointCloud<Normal>::Ptr(new PointCloud<Normal>());
		pointNormal = _pointNormal;
		planeAlong = Hyperplane<float, 3>(Vector3f(0, 1, 0), Vector3f(0, 0, 0));
		radialBand = _radialBand;
	}

	Band()
	{
		dataBand = PointCloud<PointXYZ>::Ptr(new PointCloud<PointXYZ>());
		normalBand = PointCloud<Normal>::Ptr(new PointCloud<Normal>());
		pointNormal = Vector3f(1, 0, 0);
		planeAlong = Hyperplane<float, 3>(Vector3f(0, 1, 0), Vector3f(0, 0, 0));
		radialBand = false;
	}
};

class Extractor
{
public:
	static void getNeighborsInRadius(const PointCloud<PointXYZ>::Ptr &_cloud, const PointCloud<Normal>::Ptr &_normals, const PointXYZ &_searchPoint, const double _searchRadius, PointCloud<PointXYZ>::Ptr &_surfacePatch, PointCloud<Normal>::Ptr &_normalsPatch);
	static void getNormals(const PointCloud<PointXYZ>::Ptr &_cloud, const double _searchRadius, PointCloud<Normal>::Ptr &_normals);
	static double getBands(const PointCloud<PointXYZ>::Ptr &_cloud, const PointCloud<Normal>::Ptr &_normals, const PointXYZ &_point, const Normal &_pointNormal, const ExecutionParams &_params, vector<Band> &_bands);
	static void getTangentPlane(const PointCloud<PointXYZ>::Ptr &_cloud, const PointXYZ &_point, const Normal &_pointNormal, PointCloud<PointXYZRGB>::Ptr &_tangentPlane);

private:
	Extractor();
	~Extractor();

	static double getRadialBands(const PointCloud<PointXYZ>::Ptr &_cloud, const PointCloud<Normal>::Ptr &_normals, const Vector3f &_p, const Vector3f &_n, const Hyperplane<float, 3> &_plane, const ExecutionParams &_params, vector<Band> &_bands);
	static double getLongitudinalBands(const PointCloud<PointXYZ>::Ptr &_cloud, const PointCloud<Normal>::Ptr &_normals, const Vector3f &_p, const Vector3f &_n, const Hyperplane<float, 3> &_plane, const ExecutionParams &_params, vector<Band> &_bands);
};
