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
	string sequenceMean;
	string sequenceMedian;

	Band(const PointNormal &_point, const Hyperplane<float, 3> &_plane)
	{
		data = PointCloud<PointNormal>::Ptr(new PointCloud<PointNormal>());
		point = _point;
		plane = _plane;
		isRadialBand = false;
		sequenceMean = sequenceMedian = "";
	}

	Band(const PointNormal &_point, const bool &_radialBand)
	{
		data = PointCloud<PointNormal>::Ptr(new PointCloud<PointNormal>());
		point = _point;
		plane = Hyperplane<float, 3>(Vector3f(0, 1, 0), Vector3f(0, 0, 0));
		isRadialBand = _radialBand;
		sequenceMean = sequenceMedian = "";
	}

	Band()
	{
		data = PointCloud<PointNormal>::Ptr(new PointCloud<PointNormal>());
		point = Factory::makePointNormal(1, 0, 0, 1, 0, 0);
		plane = Hyperplane<float, 3>(Vector3f(0, 1, 0), Vector3f(0, 0, 0));
		isRadialBand = false;
		sequenceMean = sequenceMedian = "";
	}
};
typedef boost::shared_ptr<Band> BandPtr;

class Extractor
{
public:
	static PointCloud<PointNormal>::Ptr getNeighbors(const PointCloud<PointNormal>::Ptr &_cloud, const PointNormal &_searchPoint, const double _searchRadius);
	static vector<BandPtr> getBands(const PointCloud<PointNormal>::Ptr &_cloud, const PointNormal &_point, const ExecutionParams &_params);
	static PointCloud<PointXYZRGB>::Ptr getTangentPlane(const PointCloud<PointNormal>::Ptr &_cloud, const PointNormal &_point);
	static vector<PointCloud<PointNormal>::Ptr> getBandPlanes(const vector<BandPtr> &_bands, const ExecutionParams &_params);

private:
	Extractor();
	~Extractor();

	static vector<BandPtr> getRadialBands(const PointCloud<PointNormal>::Ptr &_cloud, const PointNormal &_point, const ExecutionParams &_params);
	static vector<BandPtr> getLongitudinalBands(const PointCloud<PointNormal>::Ptr &_cloud, const PointNormal &_point, const ExecutionParams &_params);
};
