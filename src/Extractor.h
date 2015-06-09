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
#include "Factory.h"

using namespace std;
using namespace pcl;
using namespace Eigen;

struct Band
{
	PointCloud<PointNormal>::Ptr data;
	PointNormal point;
	Hyperplane<float, 3> plane;
	bool isRadialBand;

	Band(const PointNormal &_point, const Hyperplane<float, 3> &_plane)
	{
		data = PointCloud<PointNormal>::Ptr(new PointCloud<PointNormal>());
		point = _point;
		plane = _plane;
		isRadialBand = false;
	}

	Band(const PointNormal &_point, const bool &_radialBand)
	{
		data = PointCloud<PointNormal>::Ptr(new PointCloud<PointNormal>());
		point = _point;
		plane = Hyperplane<float, 3>(Vector3f(0, 1, 0), Vector3f(0, 0, 0));
		isRadialBand = _radialBand;
	}

	Band()
	{
		data = PointCloud<PointNormal>::Ptr(new PointCloud<PointNormal>());
		point = Factory::makePointNormal(1, 0, 0, 1, 0, 0);
		plane = Hyperplane<float, 3>(Vector3f(0, 1, 0), Vector3f(0, 0, 0));
		isRadialBand = false;
	}
};

class Extractor
{
public:
	static void getNeighborsInRadius(const PointCloud<PointNormal>::Ptr &_cloud, const PointNormal &_searchPoint, const double _searchRadius, PointCloud<PointNormal>::Ptr &_surfacePatch);
	static void getNormals(const PointCloud<PointXYZ>::Ptr &_cloud, PointCloud<Normal>::Ptr &_normals, const double _searchRadius = -1);
	static double getBands(const PointCloud<PointNormal>::Ptr &_cloud, const PointNormal &_point, const ExecutionParams &_params, vector<Band> &_bands);
	static PointCloud<PointXYZRGB>::Ptr getTangentPlane(const PointCloud<PointNormal>::Ptr &_cloud, const PointNormal &_point);

private:
	Extractor();
	~Extractor();

	static double getRadialBands(const PointCloud<PointNormal>::Ptr &_cloud, const PointNormal &_point, const ExecutionParams &_params, vector<Band> &_bands);
	static double getLongitudinalBands(const PointCloud<PointNormal>::Ptr &_cloud, const PointNormal &_point, const ExecutionParams &_params, vector<Band> &_bands);
};
