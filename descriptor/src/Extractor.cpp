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


pcl::PointCloud<pcl::PointNormal>::Ptr
Extractor::getNeighbors(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_,
						const pcl::PointNormal &searchPoint_,
						const double searchRadius_)
{
	pcl::PointCloud<pcl::PointNormal>::Ptr neighborhood(new pcl::PointCloud<pcl::PointNormal>());

	pcl::KdTreeFLANN<pcl::PointNormal> kdtree;
	kdtree.setInputCloud(cloud_);

	std::vector<int> pointIndices;
	std::vector<float> pointRadiusSquaredDistance;
	kdtree.radiusSearch(searchPoint_, searchRadius_, pointIndices, pointRadiusSquaredDistance);

	neighborhood->reserve(pointIndices.size());
	for (size_t i = 0; i < pointIndices.size(); i++)
		neighborhood->push_back(cloud_->points[pointIndices[i]]);

	// Copy the viewpoint (just in case it's needed afterwards)
	neighborhood->sensor_origin_ = cloud_->sensor_origin_;


	// TODO check the extraction of a continuous surface


	return neighborhood;
}

std::vector<BandPtr>
Extractor::getBands(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_,
					const pcl::PointNormal &point_,
					const DCHParams *params_)
{
	std::vector<BandPtr> bands;
	bands.reserve(params_->bandNumber);

	Eigen::Vector3f p = point_.getVector3fMap();
	Eigen::Vector3f n = ((Eigen::Vector3f) point_.getNormalVector3fMap()).normalized();
	Eigen::Hyperplane<float, 3> plane = Eigen::Hyperplane<float, 3>(n, p);
	std::pair<Eigen::Vector3f, Eigen::Vector3f> axes = Extractor::generateAxes(p, n, plane, params_->angle);


	/********** Debug **********/
	float debugLimit = DEBUG_getLimits(cloud_).second;
	if (Config::debugEnabled())
	{
		DEBUG_genPlane(plane, p, n, debugLimit, "plane", COLOR_TURQUOISE);
		DEBUG_genLine(Eigen::ParametrizedLine<float, 3>(p, axes.first), debugLimit, "x", COLOR_RED, false);
		DEBUG_genLine(Eigen::ParametrizedLine<float, 3>(p, axes.second), debugLimit, "y", COLOR_GREEN, false);
		DEBUG_genLine(Eigen::ParametrizedLine<float, 3>(p, n), debugLimit, "z", COLOR_BLUE, false);
	}
	/********** Debug **********/


	// Create the lines defining each band and also each band's longitudinal plane
	std::vector<Eigen::ParametrizedLine<float, 3> > lines;
	std::vector<Eigen::Vector3f> normals, directors;
	double angleStep = params_->getBandsAngularStep();
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
			DEBUG_genLine(lines[l],
						  debugLimit, "axis_band_" + boost::lexical_cast<std::string>(l),
						  (PointColor)Utils::palette12(l + 1),
						  false);
	/********** Debug **********/


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
					bands[j]->points->push_back(cloud_->points[i]);
			}
			else
			{
				/**
				 * This parts creates a vector going from the point's plane to the point currently being evaluated
				 * to find the band to which it belongs. Then the dot product is used to check if the vector
				 * points to the same direction that the director vector (which is a vector inside the band's plane).
				 *
				 * If the points as vector points to the same direction as the director vector, then the point belongs
				 * to that band (if not then it would belong to the opposite band).
				 */

				// Vector inside the original plane (the point's plane), defining point p1
				Eigen::Vector3f pointOnPlane = p + normals[j];

				// Vector going from a point inside the point's plane (p1) to the current target evaluation
				Eigen::Vector3f planeToPoint = point - pointOnPlane;

				// Calculate the orientation of the vector pointing from the plane to the current target point
				double orientation = lines[j].direction().dot(planeToPoint);

				// Add the point if the point is at the correct side of the plane and if it's in the band's limits
				if (orientation >= 0 && lines[j].distance(projection) <= halfBand)
					bands[j]->points->push_back(cloud_->points[i]);
			}
		}
	}

	return bands;
}

std::pair<Eigen::Vector3f, Eigen::Vector3f>
Extractor::generateAxes(const Eigen::Vector3f point_,
						const Eigen::Vector3f normal_,
						const Eigen::Hyperplane<float, 3> plane_,
						const float angle_)
{
	Eigen::Vector3f u = point_ + Eigen::Vector3f(0, -1e15, 0);
	Eigen::Vector3f v1 = plane_.projection(u).normalized();
	v1 = Eigen::AngleAxisf(angle_, normal_) * v1;
	v1.normalize();

	Eigen::Vector3f v2 = normal_.cross(v1).normalized();

	return std::make_pair(v1, v2);
}

void Extractor::DEBUG_genPlane(const Eigen::Hyperplane<float, 3> &plane_,
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
	pcl::io::savePCDFileASCII(DEBUG_DIR DEBUG_PREFIX + filename_ + CLOUD_FILE_EXTENSION, *targetPlane);
}

void Extractor::DEBUG_genLine(const Eigen::ParametrizedLine<float, 3> &line_,
							  const float limit_,
							  const std::string &filename_,
							  const PointColor &color_,
							  const bool full_)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr lineCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());

	float step = limit_ / 50;
	for (float t = (full_ ? -limit_ : 0); t <= limit_; t += step)
	{
		Eigen::Vector3f point = line_.pointAt(t);
		lineCloud->push_back(PointFactory::createPointXYZRGB(point.x(), point.y(), point.z(), color_));
	}

	pcl::io::savePCDFileASCII(DEBUG_DIR DEBUG_PREFIX + filename_ + CLOUD_FILE_EXTENSION, *lineCloud);
}

std::pair<float, float> Extractor::DEBUG_getLimits(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_)
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
