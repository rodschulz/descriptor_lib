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
#include "PointFactory.h"
#include "ExecutionParams.h"

struct Band
{
	pcl::PointCloud<pcl::PointNormal>::Ptr data;
	pcl::PointNormal point;
	Eigen::Hyperplane<float, 3> plane;
	bool isRadialBand;
	std::string sequenceString;
	std::vector<float> sequenceVector;

	Band(const pcl::PointNormal &_point, const Eigen::Hyperplane<float, 3> &_plane)
	{
		data = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>());
		point = _point;
		plane = _plane;
		isRadialBand = false;
		sequenceString = "";
	}

	Band(const pcl::PointNormal &_point, const bool &_radialBand)
	{
		data = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>());
		point = _point;
		plane = Eigen::Hyperplane<float, 3>(Eigen::Vector3f(0, 1, 0), Eigen::Vector3f(0, 0, 0));
		isRadialBand = _radialBand;
		sequenceString = "";
	}

	Band()
	{
		data = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>());
		point = PointFactory::makePointNormal(1, 0, 0, 1, 0, 0);
		plane = Eigen::Hyperplane<float, 3>(Eigen::Vector3f(0, 1, 0), Eigen::Vector3f(0, 0, 0));
		isRadialBand = false;
		sequenceString = "";
	}
};

typedef boost::shared_ptr<Band> BandPtr;

class Extractor
{
public:
	static pcl::PointCloud<pcl::PointNormal>::Ptr getNeighbors(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const pcl::PointNormal &_searchPoint, const double _searchRadius);
	static std::vector<BandPtr> getBands(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const pcl::PointNormal &_point, const ExecutionParams &_params);
	static pcl::PointCloud<pcl::PointXYZRGB>::Ptr getTangentPlane(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const pcl::PointNormal &_point);
	static std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> getBandPlanes(const std::vector<BandPtr> &_bands, const ExecutionParams &_params);

private:
	Extractor();
	~Extractor();

	static std::vector<BandPtr> getRadialBands(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const pcl::PointNormal &_point, const ExecutionParams &_params);
	static std::vector<BandPtr> getLongitudinalBands(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const pcl::PointNormal &_point, const ExecutionParams &_params);
};
