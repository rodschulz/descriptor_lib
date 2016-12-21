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
#include "Utils.hpp"
#include "Config.hpp"
#include "PointFactory.hpp"


pcl::PointCloud<pcl::PointNormal>::Ptr Extractor::getNeighbors(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_,
		const pcl::PointNormal &searchPoint_,
		const double searchRadius_)
{
	pcl::PointCloud<pcl::PointNormal>::Ptr surfacePatch(new pcl::PointCloud<pcl::PointNormal>());

	pcl::KdTreeFLANN<pcl::PointNormal> kdtree;
	kdtree.setInputCloud(cloud_);

	std::vector<int> pointIndices;
	std::vector<float> pointRadiusSquaredDistance;
	kdtree.radiusSearch(searchPoint_, searchRadius_, pointIndices, pointRadiusSquaredDistance);

	surfacePatch->reserve(pointIndices.size());
	for (size_t i = 0; i < pointIndices.size(); i++)
		surfacePatch->push_back(cloud_->points[pointIndices[i]]);

	// TODO check the extraction of a continuous surface

	return surfacePatch;
}

std::vector<BandPtr> Extractor::getBands(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_,
		const pcl::PointNormal &point_,
		const DCHParams *params_)
{
	std::vector<BandPtr> bands;
	bands.reserve(params_->bandNumber);

	Eigen::Vector3f p = point_.getVector3fMap();
	Eigen::Vector3f n = ((Eigen::Vector3f) point_.getNormalVector3fMap()).normalized();
	Eigen::Hyperplane<float, 3> plane = Eigen::Hyperplane<float, 3>(n, p);

	// Create a pair of perpendicular point from the given point to be used as axes in the plane
	std::pair<Eigen::Vector3f, Eigen::Vector3f> axes = Utils::generatePerpendicularPointsInPlane(plane, p);

	/********** Debug **********/
	float debugLimit = DEBUG_getDebugGenerationLimit(cloud_).second;
	if (Config::debugEnabled())
	{
		DEBUG_generatePointPlane(plane, p, n, debugLimit, "pointPlane", COLOR_TURQUOISE);
		DEBUG_generateExtractedLine(Eigen::ParametrizedLine<float, 3>(p, axes.first), debugLimit, "axis1", COLOR_RED);
		DEBUG_generateExtractedLine(Eigen::ParametrizedLine<float, 3>(p, axes.second), debugLimit, "axis2", COLOR_GREEN);
	}
	/********** Debug **********/

	// Angular step for bands definitions
	double angleStep = params_->getBandsAngularStep();

	// Create the lines defining each band and also each band's longitudinal plane
	std::vector<Eigen::ParametrizedLine<float, 3> > lines;
	std::vector<Eigen::Vector3f> normals, directors;
	for (int i = 0; i < params_->bandNumber; i++)
	{
		// Calculate the line's director std::vector and define the line
		directors.push_back((axes.first * cos(angleStep * i) + axes.second * sin(angleStep * i)).normalized());
		lines.push_back(Eigen::ParametrizedLine<float, 3>(p, directors.back()));

		// Calculate the normal to a plane going along the band and then define the plane
		normals.push_back(n.cross(directors.back()).normalized());
		bands.push_back(BandPtr(new Band(point_, Eigen::Hyperplane<float, 3>(normals.back(), p))));
	}

	/********** Debug **********/
	if (Config::debugEnabled())
		for (size_t l = 0; l < lines.size(); l++)
			DEBUG_generateExtractedLine(lines[l], debugLimit, "line" + boost::lexical_cast<std::string>(l), COLOR_YELLOW);
	/********** Debug **********/

	// TODO check if it's better to measure the distance between the projected point on the plane and the projection of the projection of the point over the line

	// Extracting points for each band (.52 to give a little extra room)
	double halfBand = params_->bandWidth * 0.52;
	for (size_t j = 0; j < lines.size(); j++)
	{
		for (size_t i = 0; i < cloud_->size(); i++)
		{
			Eigen::Vector3f point = cloud_->points[i].getVector3fMap();
			Eigen::Vector3f projection = plane.projection(point);

			if (params_->bidirectional)
			{
				if (lines[j].distance(projection) <= halfBand)
					bands[j]->data->push_back(cloud_->points[i]);
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
					bands[j]->data->push_back(cloud_->points[i]);
			}
		}
	}

	return bands;
}

std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> Extractor::generatePlaneClouds(const std::vector<BandPtr> &bands_,
		const DCHParams *params_)
{
	std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> planes;
	planes.reserve(bands_.size());

	float delta = params_->searchRadius;
	float begin = params_->bidirectional ? -delta : 0;
	float step = (delta - begin) / 10;

	for (size_t i = 0; i < bands_.size(); i++)
	{
		Eigen::Vector3f point = bands_[i]->point.getVector3fMap();
		Eigen::Vector3f normal = bands_[i]->plane.normal();
		planes.push_back(pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>()));

		for (float x = begin; x <= delta; x += step)
		{
			for (float y = begin; y <= delta; y += step)
			{
				for (float z = begin; z <= delta; z += step)
				{
					Eigen::Vector3f p = bands_[i]->plane.projection(point + Eigen::Vector3f(x, y, z));
					planes.back()->push_back((pcl::PointNormal) PointFactory::createPointNormal(p.x(), p.y(), p.z(), normal[0], normal[1], normal[2]));
				}
			}
		}
	}

	return planes;
}

void Extractor::DEBUG_generatePointPlane(const Eigen::Hyperplane<float, 3> &plane_,
		const Eigen::Vector3f &p_,
		const Eigen::Vector3f &n_,
		const float limit_,
		const std::string &filename_,
		const PointColor &color_)
{

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr targetPlane = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
	float step = limit_ / 15;
	for (float i = -limit_; i <= limit_; i += step)
	{
		for (float j = -limit_; j <= limit_; j += step)
		{
			for (float k = -limit_; k <= limit_; k += step)
			{
				Eigen::Vector3f aux = plane_.projection(Eigen::Vector3f(p_.x() + i, p_.y() + j, p_.z() + k));
				targetPlane->push_back(PointFactory::createPointXYZRGBNormal(aux.x(), aux.y(), aux.z(), n_.x(), n_.y(), n_.z(), 0, color_));
			}
		}
	}
	pcl::io::savePCDFileASCII(OUTPUT_DIR DEBUG_PREFIX + filename_ + CLOUD_FILE_EXTENSION, *targetPlane);
}

void Extractor::DEBUG_generateExtractedLine(const Eigen::ParametrizedLine<float, 3> &line_,
		const float limit_,
		const std::string &filename_,
		const PointColor &color_)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr lineCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());

	float step = limit_ / 50;
	for (float t = -limit_; t <= limit_; t += step)
	{
		Eigen::Vector3f point = line_.pointAt(t);
		lineCloud->push_back(PointFactory::createPointXYZRGB(point.x(), point.y(), point.z(), color_));
	}

	pcl::io::savePCDFileASCII(OUTPUT_DIR DEBUG_PREFIX + filename_ + CLOUD_FILE_EXTENSION, *lineCloud);
}

std::pair<float, float> Extractor::DEBUG_getDebugGenerationLimit(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_)
{
	pcl::PointNormal minData, maxData;
	pcl::getMinMax3D(*cloud_, minData, maxData);

	float deltaX = maxData.x - minData.x;
	float deltaY = maxData.y - minData.y;
	float deltaZ = maxData.z - minData.z;

	float data[] =
	{ deltaX, deltaY, deltaZ };
	std::pair<float *, float *> minmax = boost::minmax_element(&data[0], &data[3]);

	return std::pair<float, float>((*minmax.first) / 2, (*minmax.second) / 2);
}
