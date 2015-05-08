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

	// Create a pair of perpendicular vectors to be used as axes in the plane
	Vector3f d1(p[0] + 10, p[1], p[2]);
	d1 = plane.projection(d1);
	d1.normalize();
	Vector3f d2 = n.cross(d1);
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
		lines.push_back(ParametrizedLine<float, 3>(p, director));

		// Calculate the plane's normal and then define the plane
		Vector3f bandPlaneNormal = n.cross(director);
		bandPlaneNormal.normalize();
		_bands.push_back(Band(n, Hyperplane<float, 3>(bandPlaneNormal, p)));
		planeNormals.push_back(bandPlaneNormal);
	}

	// Start extracting points
	double halfBand = _params.bandWidth * 0.5;
	for (size_t i = 0; i < _cloud->size(); i++)
	{
		Vector3f point = _cloud->points[i].getVector3fMap();
		Vector3f projection = plane.projection(point);

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
				Vector3f pointOnPlane = p + planeNormals[j];
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
