/**
 * Author: rodrigo
 * 2015
 */
#include "Extractor.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <eigen3/Eigen/src/Geometry/ParametrizedLine.h>
#include "PointFactory.h"

Extractor::Extractor()
{
}

Extractor::~Extractor()
{
}

pcl::PointCloud<pcl::PointNormal>::Ptr Extractor::getNeighbors(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const pcl::PointNormal &_searchPoint, const double _searchRadius)
{
	pcl::PointCloud<pcl::PointNormal>::Ptr surfacePatch(new pcl::PointCloud<pcl::PointNormal>());

	pcl::KdTreeFLANN<pcl::PointNormal> kdtree;
	kdtree.setInputCloud(_cloud);

	std::vector<int> pointIndices;
	std::vector<float> pointRadiusSquaredDistance;
	kdtree.radiusSearch(_searchPoint, _searchRadius, pointIndices, pointRadiusSquaredDistance);

	surfacePatch->reserve(pointIndices.size());
	for (size_t i = 0; i < pointIndices.size(); i++)
		surfacePatch->push_back(_cloud->points[pointIndices[i]]);

	return surfacePatch;
}

std::vector<BandPtr> Extractor::getBands(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const pcl::PointNormal &_point, const ExecutionParams &_params)
{
	return getLongitudinalBands(_cloud, _point, _params);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Extractor::getTangentPlane(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const pcl::PointNormal &_point)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tangentPlane(new pcl::PointCloud<pcl::PointXYZRGB>());

	Eigen::Vector3f p = _point.getVector3fMap();
	Eigen::Vector3f n = _point.getNormalVector3fMap();
	Eigen::Hyperplane<float, 3> plane = Eigen::Hyperplane<float, 3>(n, p);

	tangentPlane->clear();
	tangentPlane->reserve(_cloud->size());
	for (size_t i = 0; i < _cloud->size(); i++)
	{
		Eigen::Vector3f p = _cloud->points[i].getVector3fMap();
		Eigen::Vector3f projection = plane.projection(p);
		tangentPlane->push_back(PointFactory::makePointXYZRGB((float) projection[0], (float) projection[1], (float) projection[2], 0, 0, 255));
	}

	return tangentPlane;
}

std::vector<BandPtr> Extractor::getLongitudinalBands(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const pcl::PointNormal &_point, const ExecutionParams &_params)
{
	std::vector<BandPtr> bands;
	bands.reserve(_params.bandNumber);

	Eigen::Vector3f p = _point.getVector3fMap();
	Eigen::Vector3f n = _point.getNormalVector3fMap();
	Eigen::Hyperplane<float, 3> plane = Eigen::Hyperplane<float, 3>(n, p);

	// Create a pair of perpendicular vectors to be used as axes in the plane
	Eigen::Vector3f v1 = p + Eigen::Vector3f(10, 10, 10); // TODO find a better way to get this initial director std::vector
	v1 = plane.projection(v1).normalized();
	Eigen::Vector3f v2 = n.cross(v1).normalized();

	// Angular step for bands definitions
	double angleStep = _params.getBandsAngularStep();

	// Create the lines defining each band and also each band's longitudinal plane
	std::vector<Eigen::ParametrizedLine<float, 3> > lines;
	std::vector<Eigen::Vector3f> normals, directors;
	for (int i = 0; i < _params.bandNumber; i++)
	{
		// Calculate the line's director std::vector and define the line
		Eigen::Vector3f director = v1 * cos(angleStep * i) + v2 * sin(angleStep * i);
		directors.push_back(director);
		lines.push_back(Eigen::ParametrizedLine<float, 3>(p, director));

		// Calculate the normal to a plane going along the band and then define the plane
		Eigen::Vector3f planeNormal = n.cross(director).normalized();
		bands.push_back(BandPtr(new Band(_point, Eigen::Hyperplane<float, 3>(planeNormal, p))));
		normals.push_back(planeNormal);
	}

	// Start extracting points
	double halfBand = _params.bandWidth * 0.5;
	for (size_t i = 0; i < _cloud->size(); i++)
	{
		Eigen::Vector3f point = _cloud->points[i].getVector3fMap();
		Eigen::Vector3f projection = plane.projection(point);

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
				// product to check if the std::vector points the same direction than the director std::vector
				Eigen::Vector3f pointOnPlane = p + normals[j];
				Eigen::Vector3f planeToPoint = point - pointOnPlane;
				double orientation = directors[j].dot(planeToPoint);

				if (orientation >= 0 && lines[j].distance(projection) <= halfBand)
					bands[j]->data->push_back(_cloud->points[i]);
			}
		}
	}

	return bands;
}

std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> Extractor::getBandPlanes(const std::vector<BandPtr> &_bands, const ExecutionParams &_params)
{
	std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> planes;
	planes.reserve(_bands.size());

	float delta = _params.patchSize;
	float begin = _params.bidirectional ? -delta : 0;
	float step = (delta - begin) / 10;

	for (size_t i = 0; i < _bands.size(); i++)
	{
		Eigen::Vector3f point = _bands[i]->point.getVector3fMap();
		Eigen::Vector3f normal = _bands[i]->plane.normal();
		planes.push_back(pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>()));

		for (float x = begin; x <= delta; x += step)
		{
			for (float y = begin; y <= delta; y += step)
			{
				for (float z = begin; z <= delta; z += step)
				{
					Eigen::Vector3f p = _bands[i]->plane.projection(point + Eigen::Vector3f(x, y, z));
					planes.back()->push_back((pcl::PointNormal) PointFactory::makePointNormal(p.x(), p.y(), p.z(), normal[0], normal[1], normal[2]));
				}
			}
		}
	}

	return planes;
}
