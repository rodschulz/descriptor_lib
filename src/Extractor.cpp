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

PointCloud<PointNormal>::Ptr Extractor::getNeighborsInRadius(const PointCloud<PointNormal>::Ptr &_cloud, const PointNormal &_searchPoint, const double _searchRadius)
{
	PointCloud<PointNormal>::Ptr surfacePatch(new PointCloud<PointNormal>());

	KdTreeFLANN<PointNormal> kdtree;
	kdtree.setInputCloud(_cloud);

	vector<int> pointIndices;
	vector<float> pointRadiusSquaredDistance;
	kdtree.radiusSearch(_searchPoint, _searchRadius, pointIndices, pointRadiusSquaredDistance);

	surfacePatch->reserve(pointIndices.size());
	for (size_t i = 0; i < pointIndices.size(); i++)
		surfacePatch->push_back(_cloud->points[pointIndices[i]]);

	return surfacePatch;
}

vector<BandPtr> Extractor::getBands(const PointCloud<PointNormal>::Ptr &_cloud, const PointNormal &_point, const ExecutionParams &_params)
{
	if (_params.radialBands)
		return getRadialBands(_cloud, _point, _params);
	else
		return getLongitudinalBands(_cloud, _point, _params);
}

PointCloud<PointXYZRGB>::Ptr Extractor::getTangentPlane(const PointCloud<PointNormal>::Ptr &_cloud, const PointNormal &_point)
{
	PointCloud<PointXYZRGB>::Ptr tangentPlane(new PointCloud<PointXYZRGB>());

	Vector3f p = _point.getVector3fMap();
	Vector3f n = _point.getNormalVector3fMap();
	Hyperplane<float, 3> plane = Hyperplane<float, 3>(n, p);

	tangentPlane->clear();
	tangentPlane->reserve(_cloud->size());
	for (size_t i = 0; i < _cloud->size(); i++)
	{
		Vector3f p = _cloud->points[i].getVector3fMap();
		Vector3f projection = plane.projection(p);
		tangentPlane->push_back(Factory::makePointXYZRGB((float) projection[0], (float) projection[1], (float) projection[2], 0, 0, 255));
	}

	return tangentPlane;
}

void Extractor::getSequences(const vector<BandPtr> &_bands)
{
}

vector<BandPtr> Extractor::getRadialBands(const PointCloud<PointNormal>::Ptr &_cloud, const PointNormal &_point, const ExecutionParams &_params)
{
	vector<BandPtr> bands;
	bands.reserve(_params.bandNumber);

	// Get the point, its normal and make a plane tangent to the point
	Vector3f p = _point.getVector3fMap();
	Vector3f n = _point.getNormalVector3fMap();
	Hyperplane<float, 3> plane = Hyperplane<float, 3>(n, p);

	double radialStep = _params.searchRadius / _params.bandNumber;

	// Create bands
	for (int i = 0; i < _params.bandNumber; i++)
		bands.push_back(BandPtr(new Band(_point, true)));

	// Extract points
	for (size_t i = 0; i < _cloud->size(); i++)
	{
		Vector3f point = _cloud->points[i].getVector3fMap();
		Vector3f projection = plane.projection(point);

		Vector3f diff = p - projection;
		double distance = diff.norm();
		int index = (int) (distance / radialStep);

		bands[index]->data->push_back(_cloud->points[i]);
	}

	return bands;
}

vector<BandPtr> Extractor::getLongitudinalBands(const PointCloud<PointNormal>::Ptr &_cloud, const PointNormal &_point, const ExecutionParams &_params)
{
	vector<BandPtr> bands;
	bands.reserve(_params.bandNumber);

	Vector3f p = _point.getVector3fMap();
	Vector3f n = _point.getNormalVector3fMap();
	Hyperplane<float, 3> plane = Hyperplane<float, 3>(n, p);

	// Create a pair of perpendicular vectors to be used as axes in the plane
	Vector3f v1 = p + Vector3f(10, 10, 10); // TODO find a better way to get this initial director vector
	v1 = plane.projection(v1).normalized();
	Vector3f v2 = n.cross(v1).normalized();

	// Angular step for bands definitions
	double angleRange = _params.bidirectional ? M_PI : 2 * M_PI;
	double angleStep = angleRange / _params.bandNumber;

	// Create the lines defining each band and also each band's longitudinal plane
	vector<ParametrizedLine<float, 3> > lines;
	vector<Vector3f> normals, directors;
	for (int i = 0; i < _params.bandNumber; i++)
	{
		// Calculate the line's director vector and define the line
		Vector3f director = v1 * cos(angleStep * i) + v2 * sin(angleStep * i);
		directors.push_back(director);
		lines.push_back(ParametrizedLine<float, 3>(p, director));

		// Calculate the normal to a plane going along the band and then define the plane
		Vector3f planeNormal = n.cross(director).normalized();
		bands.push_back(BandPtr(new Band(_point, Hyperplane<float, 3>(planeNormal, p))));
		normals.push_back(planeNormal);
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
					bands[j]->data->push_back(_cloud->points[i]);
			}
			else
			{
				// Create a vector going from the plane to the current point and the use the dot
				// product to check if the vector points the same direction than the director vector
				Vector3f pointOnPlane = p + normals[j];
				Vector3f planeToPoint = point - pointOnPlane;
				double orientation = directors[j].dot(planeToPoint);

				if (orientation >= 0 && lines[j].distance(projection) <= halfBand)
					bands[j]->data->push_back(_cloud->points[i]);
			}
		}
	}

	return bands;
}
