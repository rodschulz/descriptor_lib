/**
 * Author: rodrigo
 * 2015
 */
#include "Extractor.hpp"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/impl/common.hpp>
#include <eigen3/Eigen/src/Geometry/ParametrizedLine.h>
#include <boost/algorithm/minmax_element.hpp>
#include "../utils/Utils.hpp"
#include "../utils/Config.hpp"

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
	std::vector<BandPtr> bands;
	bands.reserve(_params.bandNumber);

	Eigen::Vector3f p = _point.getVector3fMap();
	Eigen::Vector3f n = ((Eigen::Vector3f) _point.getNormalVector3fMap()).normalized();
	Eigen::Hyperplane<float, 3> plane = Eigen::Hyperplane<float, 3>(n, p);

	// Create a pair of perpendicular point from the given point to be used as axes in the plane
	std::pair<Eigen::Vector3f, Eigen::Vector3f> axes = Utils::generatePerpendicularPointsInPlane(plane, p);

	/********** Debug **********/
	float debugLimit = DEBUG_getDebugGenerationLimit(_cloud).second;
	if (Config::debugEnabled())
	{
		DEBUG_generatePointPlane(plane, p, n, debugLimit, "pointPlane", COLOR_TURQUOISE);
		DEBUG_generateExtractedLine(Eigen::ParametrizedLine<float, 3>(p, axes.first), debugLimit, "axis1", COLOR_RED);
		DEBUG_generateExtractedLine(Eigen::ParametrizedLine<float, 3>(p, axes.second), debugLimit, "axis2", COLOR_GREEN);
	}
	/********** Debug **********/

	// Angular step for bands definitions
	double angleStep = _params.getBandsAngularStep();

	// Create the lines defining each band and also each band's longitudinal plane
	std::vector<Eigen::ParametrizedLine<float, 3> > lines;
	std::vector<Eigen::Vector3f> normals, directors;
	for (int i = 0; i < _params.bandNumber; i++)
	{
		// Calculate the line's director std::vector and define the line
		directors.push_back((axes.first * cos(angleStep * i) + axes.second * sin(angleStep * i)).normalized());
		lines.push_back(Eigen::ParametrizedLine<float, 3>(p, directors.back()));

		// Calculate the normal to a plane going along the band and then define the plane
		normals.push_back(n.cross(directors.back()).normalized());
		bands.push_back(BandPtr(new Band(_point, Eigen::Hyperplane<float, 3>(normals.back(), p))));
	}

	/********** Debug **********/
	if (Config::debugEnabled())
		for (size_t l = 0; l < lines.size(); l++)
			DEBUG_generateExtractedLine(lines[l], debugLimit, "line" + boost::lexical_cast<std::string>(l), COLOR_YELLOW);
	/********** Debug **********/

	// Extracting points for each band (.52 to give a little extra room)
	double halfBand = _params.bandWidth * 0.52;
	for (size_t j = 0; j < lines.size(); j++)
	{
		for (size_t i = 0; i < _cloud->size(); i++)
		{
			Eigen::Vector3f point = _cloud->points[i].getVector3fMap();
			Eigen::Vector3f projection = plane.projection(point);

			if (_params.bidirectional)
			{
				if (lines[j].distance(projection) <= halfBand)
					bands[j]->data->push_back(_cloud->points[i]);
			}
			else
			{
				/**
				 * This parts creates a vector going from the point's plane to the point currently being evaluated
				 * to find the band to which it belongs. Then the dot product is used to check if the vector
				 * points to the same direction that the director vector (which is a vector inside the band's plane).
				 *
				 * If the points as vector points to the same direction as the director vector, then the point belongs
				 * to that band (if not then it would belong to the oppposite band).
				 */

				// Vector inside the original plane (the point's plane), defining point p1
				Eigen::Vector3f pointOnPlane = p + normals[j];

				// Vector going from a point inside the point's plane (p1) to the current target evaluation
				Eigen::Vector3f planeToPoint = point - pointOnPlane;

				// Calculate the orientation of the vector pointing from the plane to the current target point
				double orientation = lines[j].direction().dot(planeToPoint);

				// Add the point if the point is at the correct side of the plane and if it's in the band's limits
				if (orientation >= 0 && lines[j].distance(projection) <= halfBand)
					bands[j]->data->push_back(_cloud->points[i]);
			}
		}
	}

	return bands;
}

std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> Extractor::generatePlaneClouds(const std::vector<BandPtr> &_bands, const ExecutionParams &_params)
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
					planes.back()->push_back((pcl::PointNormal) PointFactory::createPointNormal(p.x(), p.y(), p.z(), normal[0], normal[1], normal[2]));
				}
			}
		}
	}

	return planes;
}

void Extractor::DEBUG_generatePointPlane(const Eigen::Hyperplane<float, 3> &_plane, const Eigen::Vector3f &_p, const Eigen::Vector3f &_n, const float _limit, const std::string &_filename, const PointColor &_color)
{

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr targetPlane = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
	float step = _limit / 15;
	for (float i = -_limit; i <= _limit; i += step)
	{
		for (float j = -_limit; j <= _limit; j += step)
		{
			for (float k = -_limit; k <= _limit; k += step)
			{
				Eigen::Vector3f aux = _plane.projection(Eigen::Vector3f(_p.x() + i, _p.y() + j, _p.z() + k));
				targetPlane->push_back(PointFactory::createPointXYZRGBNormal(aux.x(), aux.y(), aux.z(), _n.x(), _n.y(), _n.z(), 0, _color));
			}
		}
	}
	pcl::io::savePCDFileASCII(OUTPUT_FOLDER DEBUG_PREFIX + _filename + CLOUD_FILE_EXTENSION, *targetPlane);
}

void Extractor::DEBUG_generateExtractedLine(const Eigen::ParametrizedLine<float, 3> &_line, const float _limit, const std::string &_filename, const PointColor &_color)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr lineCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());

	float step = _limit / 50;
	for (float t = -_limit; t <= _limit; t += step)
	{
		Eigen::Vector3f point = _line.pointAt(t);
		lineCloud->push_back(PointFactory::createPointXYZRGB(point.x(), point.y(), point.z(), _color));
	}

	pcl::io::savePCDFileASCII(OUTPUT_FOLDER DEBUG_PREFIX + _filename + CLOUD_FILE_EXTENSION, *lineCloud);
}

std::pair<float, float> Extractor::DEBUG_getDebugGenerationLimit(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud)
{
	pcl::PointNormal minData, maxData;
	pcl::getMinMax3D(*_cloud, minData, maxData);

	float deltaX = maxData.x - minData.x;
	float deltaY = maxData.y - minData.y;
	float deltaZ = maxData.z - minData.z;

	float data[] =
	{ deltaX, deltaY, deltaZ };
	std::pair<float *, float *> minmax = boost::minmax_element(&data[0], &data[3]);

	return std::pair<float, float>((*minmax.first) / 2, (*minmax.second) / 2);
}
