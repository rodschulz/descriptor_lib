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

double Extractor::getBands(const PointCloud<PointXYZ>::Ptr &_cloud, const PointCloud<Normal>::Ptr &_normals, const PointXYZ &_point, const Normal &_pointNormal, const ExecutionParams &_params, vector<Band> &_bands)
{
	// Get the point, its normal and make a plane
	Vector3f p = _point.getVector3fMap();
	Vector3f n = _pointNormal.getNormalVector3fMap();
	Hyperplane<float, 3> plane = Hyperplane<float, 3>(n, p);

	double step;
	if (_params.radialBands)
		step = getRadialBands(_cloud, _normals, p, n, plane, _params, _bands);
	else
		step = getLongitudinalBands(_cloud, _normals, p, n, plane, _params, _bands);

	return step;
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

double Extractor::getRadialBands(const PointCloud<PointXYZ>::Ptr &_cloud, const PointCloud<Normal>::Ptr &_normals, const Vector3f &_p, const Vector3f &_n, const Hyperplane<float, 3> &_plane, const ExecutionParams &_params, vector<Band> &_bands)
{
	double radialStep = _params.searchRadius / _params.bandNumber;

	// Create bands
	_bands.clear();
	for (int i = 0; i < _params.bandNumber; i++)
		_bands.push_back(Band(_n, true));

	// Extract points
	for (size_t i = 0; i < _cloud->size(); i++)
	{
		Vector3f point = _cloud->points[i].getVector3fMap();
		Vector3f projection = _plane.projection(point);

		Vector3f diff = _p - projection;
		double distance = diff.norm();
		int index = (int) (distance / radialStep);

		_bands[index].dataBand->push_back(_cloud->points[i]);
		_bands[index].normalBand->push_back(_normals->points[i]);
	}

	return radialStep;
}

double Extractor::getLongitudinalBands(const PointCloud<PointXYZ>::Ptr &_cloud, const PointCloud<Normal>::Ptr &_normals, const Vector3f &_p, const Vector3f &_n, const Hyperplane<float, 3> &_plane, const ExecutionParams &_params, vector<Band> &_bands)
{
	// Create a pair of perpendicular vectors to be used as axes in the plane
	Vector3f d1(_p[0] + 10, _p[1], _p[2]);
	d1 = _plane.projection(d1);
	d1.normalize();
	Vector3f d2 = _n.cross(d1);
	d2.normalize();

	// Angular step for bands definitions
	double angleRange = _params.bidirectional ? M_PI : 2 * M_PI;
	double angleStep = angleRange / _params.bandNumber;

	// Create the lines defining each band and also each band's longitudinal plane
	_bands.clear();
	vector<ParametrizedLine<float, 3> > lines;
	vector<Vector3f> planeNormals;
	vector<Vector3f> directors;
	for (int i = 0; i < _params.bandNumber; i++)
	{
		// Calculate the line's director vector and define the line
		Vector3f director = d1 * cos(angleStep * i) + d2 * sin(angleStep * i);
		directors.push_back(director);
		lines.push_back(ParametrizedLine<float, 3>(_p, director));

		// Calculate the plane's normal and then define the plane
		Vector3f bandPlaneNormal = _n.cross(director);
		bandPlaneNormal.normalize();
		_bands.push_back(Band(_n, Hyperplane<float, 3>(bandPlaneNormal, _p)));
		planeNormals.push_back(bandPlaneNormal);
	}

	// Start extracting points
	double halfBand = _params.bandWidth * 0.5;
	for (size_t i = 0; i < _cloud->size(); i++)
	{
		Vector3f point = _cloud->points[i].getVector3fMap();
		Vector3f projection = _plane.projection(point);

		for (size_t j = 0; j < lines.size(); j++)
		{
			if (_params.bidirectional)
			{
				if (lines[j].distance(projection) <= halfBand)
				{
					_bands[j].dataBand->push_back(_cloud->points[i]);
					_bands[j].normalBand->push_back(_normals->points[i]);
				}
			}
			else
			{
				// Create a vector going from the plane to the current point and the use the dot
				// product to check if the vector points the same direction than the director vector
				Vector3f pointOnPlane = _p + planeNormals[j];
				Vector3f planeToPoint = point - pointOnPlane;
				double orientation = directors[j].dot(planeToPoint);

				if (orientation >= 0 && lines[j].distance(projection) <= halfBand)
				{
					_bands[j].dataBand->push_back(_cloud->points[i]);
					_bands[j].normalBand->push_back(_normals->points[i]);
				}
			}
		}
	}

	return angleStep;
}
