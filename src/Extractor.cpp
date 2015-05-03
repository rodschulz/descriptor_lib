/**
 * Author: rodrigo
 * 2015
 */
#include "Extractor.h"
#include "Factory.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <eigen3/Eigen/src/Geometry/ParametrizedLine.h>

Extractor::Extractor()
{
}

Extractor::~Extractor()
{
}

void Extractor::getNeighborsInRadius(const PointCloud<PointXYZ>::Ptr &_cloud, const PointCloud<Normal>::Ptr &_normals, const PointXYZ &_searchPoint, const double _searchRadius, PointCloud<PointXYZ>::Ptr &_surfacePatch, PointCloud<Normal>::Ptr &_normalsPatch)
{
	KdTreeFLANN<PointXYZ> kdtree;
	kdtree.setInputCloud(_cloud);

	vector<int> pointIndices;
	vector<float> pointRadiusSquaredDistance;
	kdtree.radiusSearch(_searchPoint, _searchRadius, pointIndices, pointRadiusSquaredDistance);

	_surfacePatch->clear();
	_surfacePatch->reserve(pointIndices.size());
	_normalsPatch->clear();
	_normalsPatch->reserve(pointIndices.size());

	for (size_t i = 0; i < pointIndices.size(); i++)
	{
		_surfacePatch->push_back(_cloud->points[pointIndices[i]]);
		_normalsPatch->push_back(_normals->points[pointIndices[i]]);
	}
}

void Extractor::getNormals(const PointCloud<PointXYZ>::Ptr &_cloud, const double _searchRadius, PointCloud<Normal>::Ptr &_normals)
{
	// Search method used for the knn search
	search::KdTree<PointXYZ>::Ptr kdtree(new search::KdTree<PointXYZ>);

	NormalEstimation<PointXYZ, Normal> normalEstimation;
	normalEstimation.setInputCloud(_cloud);
	normalEstimation.setRadiusSearch(_searchRadius);
	normalEstimation.setSearchMethod(kdtree);
	normalEstimation.compute(*_normals);
}

void Extractor::getBands(const PointCloud<PointXYZ>::Ptr &_cloud, const PointCloud<Normal>::Ptr &_normals, const PointXYZ &_point, const Normal &_pointNormal, const double _bandWidth, vector<Band> &_bands)
{
	// Get the point, its normal and make a plane
	Vector3f p = _point.getVector3fMap();
	Vector3f n = _pointNormal.getNormalVector3fMap();
	Hyperplane<float, 3> plane = Hyperplane<float, 3>(n, p);

	// Create a pair of perpendicular vectors to be used as axes in the plane
	Vector3f d1(p[0] + 10, p[1], p[2]);
	d1 = plane.projection(d1);
	d1.normalize();
	Vector3f d2 = n.cross(d1);
	d2.normalize();

	// Create diagonal unit vectors to create diagonal lines
	Vector3f diag45 = d1 + d2;
	diag45.normalize();
	Vector3f diag135 = d2 - d1;
	diag135.normalize();

	// Directions to extract points
	ParametrizedLine<float, 3> line0 = ParametrizedLine<float, 3>(p, d1);
	ParametrizedLine<float, 3> line45 = ParametrizedLine<float, 3>(p, diag45);
	ParametrizedLine<float, 3> line90 = ParametrizedLine<float, 3>(p, d2);
	ParametrizedLine<float, 3> line135 = ParametrizedLine<float, 3>(p, diag135);

	// Calculate planes going along each band
	Vector3f normalDiag45 = n.cross(diag45);
	normalDiag45.normalize();
	Vector3f normalDiag135 = n.cross(diag135);
	normalDiag135.normalize();
	Hyperplane<float, 3> planeAlong0 = Hyperplane<float, 3>(d2, p);
	Hyperplane<float, 3> planeAlong45 = Hyperplane<float, 3>(normalDiag45, p);
	Hyperplane<float, 3> planeAlong90 = Hyperplane<float, 3>(d1, p);
	Hyperplane<float, 3> planeAlong135 = Hyperplane<float, 3>(normalDiag135, p);

	// Start extracting points
	_bands.clear();
	_bands.push_back(Band(n, planeAlong0));
	_bands.push_back(Band(n, planeAlong45));
	_bands.push_back(Band(n, planeAlong90));
	_bands.push_back(Band(n, planeAlong135));

	double halfBand = _bandWidth * 0.5;
	for (size_t i = 0; i < _cloud->size(); i++)
	{
		Vector3f point = _cloud->points[i].getVector3fMap();
		Vector3f projection = plane.projection(point);

		if (line0.distance(projection) <= halfBand)
		{
			_bands[0].dataBand->push_back(_cloud->points[i]);
			_bands[0].normalBand->push_back(_normals->points[i]);
		}

		if (line45.distance(projection) <= halfBand)
		{
			_bands[1].dataBand->push_back(_cloud->points[i]);
			_bands[1].normalBand->push_back(_normals->points[i]);
		}

		if (line90.distance(projection) <= halfBand)
		{
			_bands[2].dataBand->push_back(_cloud->points[i]);
			_bands[2].normalBand->push_back(_normals->points[i]);
		}

		if (line135.distance(projection) <= halfBand)
		{
			_bands[3].dataBand->push_back(_cloud->points[i]);
			_bands[3].normalBand->push_back(_normals->points[i]);
		}
	}
}

void Extractor::getTangentPlane(const PointCloud<PointXYZ>::Ptr &_cloud, const PointXYZ &_point, const Normal &_pointNormal, PointCloud<PointXYZRGB>::Ptr &_tangentPlane)
{
	Vector3f p = _point.getVector3fMap();
	Vector3f n = _pointNormal.getNormalVector3fMap();
	Hyperplane<float, 3> plane = Hyperplane<float, 3>(n, p);

	_tangentPlane->clear();
	_tangentPlane->reserve(_cloud->size());
	for (size_t i = 0; i < _cloud->size(); i++)
	{
		Vector3f p = _cloud->points[i].getVector3fMap();
		Vector3f projection = plane.projection(p);
		_tangentPlane->push_back(Factory::makePointXYZRGB((float) projection[0], (float) projection[1], (float) projection[2], 0, 0, 255));
	}
}
