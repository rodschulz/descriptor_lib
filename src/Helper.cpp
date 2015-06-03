/**
 * Author: rodrigo
 * 2015
 */
#include "Helper.h"
#include <pcl/filters/filter.h>
#include <ctype.h>
#include "Factory.h"

Helper::Helper()
{
}

Helper::~Helper()
{
}

void Helper::removeNANs(PointCloud<PointXYZ>::Ptr &_cloud)
{
	std::vector<int> mapping;
	removeNaNFromPointCloud(*_cloud, *_cloud, mapping);
}

PointCloud<PointXYZRGB>::Ptr Helper::createColorCloud(const PointCloud<PointXYZ>::Ptr &_cloud, const uint8_t _r, const uint8_t _g, const uint8_t _b)
{
	PointCloud<PointXYZRGB>::Ptr coloredCloud(new PointCloud<PointXYZRGB>());
	coloredCloud->reserve(_cloud->size());

	for (int i = 0; i < _cloud->width; i++)
		coloredCloud->push_back(Factory::makePointXYZRGB(_cloud->points[i].x, _cloud->points[i].y, _cloud->points[i].z, _r, _g, _b));

	return coloredCloud;
}

float Helper::getColor(const uint8_t _r, const uint8_t _g, const uint8_t _b)
{
	uint32_t color = ((uint32_t) _r << 16 | (uint32_t) _g << 8 | (uint32_t) _b);
	float finalColor = *reinterpret_cast<float*>(&color);
	return finalColor;
}

void Helper::calculateAngleHistograms(const vector<Band> &_bands, const PointNormal &_point, vector<Hist> &_histograms)
{
	_histograms.clear();
	_histograms.reserve(_bands.size());

	Vector3f targetPoint = _point.getVector3fMap();
	Vector3f targetNormal = _point.getNormalVector3fMap();

	for (size_t i = 0; i < _bands.size(); i++)
	{
		_histograms.push_back(Hist(ANGLE));
		for (size_t j = 0; j < _bands[i].data->size(); j++)
		{
			Vector3f point = _bands[i].data->points[j].getVector3fMap();
			Vector3f normal = _bands[i].data->points[j].getNormalVector3fMap();

			if (_bands[i].isRadialBand)
			{
				// Create a plane containing the target point, its normal and the current point
				Vector3f targetToPoint = point - targetPoint;

				// Skip if the point is equal to the target point
				if (fabs(targetToPoint.norm()) < 1E-15)
					continue;

				Vector3f planeNormal = targetNormal.cross(targetToPoint).normalized();
				Hyperplane<float, 3> plane = Hyperplane<float, 3>(planeNormal, targetPoint);

				// Project current point's normal onto the plane and calculate normal's angle
				Vector3f projectedNormal = plane.projection(normal).normalized();
				double theta = angleBetween<Vector3f>(targetNormal, projectedNormal);
				_histograms.back().add(theta);
			}
			else
			{
				// TODO I think this angle should be against the original normal, not the projected one
				Vector3f projectedNormal = _bands[i].plane.projection(normal).normalized();
				double theta = angleBetween<Vector3f>(targetNormal, projectedNormal);
				//double theta = angleBetween<Vector3f>(targetNormal, normal);
				_histograms.back().add(theta);
			}
		}
	}
}

void Helper::calculateCurvatureHistograms(const vector<Band> &_bands, const PointNormal &_point, vector<Hist> &_histograms)
{
	_histograms.clear();
	_histograms.reserve(_bands.size());

	Vector3f targetPoint = _point.getVector3fMap();
	Vector3f targetNormal = _point.getNormalVector3fMap();

	for (size_t i = 0; i < _bands.size(); i++)
	{
		Hist histogram;

		for (size_t j = 0; j < _bands[i].data->size(); j++)
		{
			Vector3f point = _bands[i].data->points[j].getVector3fMap();
			Vector3f normal = _bands[i].data->points[j].getNormalVector3fMap();

			if (_bands[i].isRadialBand)
			{
				// Create a plane containing the target point, its normal and the current point
				Vector3f targetToPoint = targetPoint - point;

				// Skip if the point is equal to the target point
				if (fabs(targetToPoint.norm()) < 1E-15)
					continue;

				Vector3f planeNormal = targetNormal.cross(targetToPoint);
				planeNormal.normalize();
				Hyperplane<float, 3> plane = Hyperplane<float, 3>(planeNormal, targetPoint);

				// Project current point's normal onto the plane and get the point's curvature
				Vector3f projectedNormal = plane.projection(normal);

				Vector3f pointDiff = targetPoint - point;
				double pointDistance = pointDiff.norm();
				Vector3f normalDiff = targetNormal - projectedNormal;
				double normalDistance = normalDiff.norm();

				if (pointDistance > 0)
					histogram.add(normalDistance / pointDistance);
			}
			else
			{
				Vector3f projectedPoint = _bands[i].plane.projection(point);
				Vector3f projectedNormal = _bands[i].plane.projection(normal);

				Vector3f pointDiff = targetPoint - projectedPoint;
				double pointDistance = pointDiff.norm();
				Vector3f normalDiff = targetNormal - projectedNormal;
				double normalDistance = normalDiff.norm();

				if (pointDistance > 0)
					histogram.add(normalDistance / pointDistance);
			}
		}

		_histograms.push_back(histogram);
	}
}

void Helper::calculateMeanCurvature(const vector<Band> &_bands, const PointNormal &_point, vector<double> &_curvatures)
{
	_curvatures.clear();
	_curvatures.reserve(_bands.size());

	Vector3f targetPoint = _point.getVector3fMap();
	Vector3f targetNormal = _point.getNormalVector3fMap();

	for (size_t i = 0; i < _bands.size(); i++)
	{
		double curvature = 0;
		for (size_t j = 0; j < _bands[i].data->size(); j++)
		{
			Vector3f point = _bands[i].data->points[j].getVector3fMap();
			Vector3f normal = _bands[i].data->points[j].getNormalVector3fMap();

			if (_bands[i].isRadialBand)
			{
				// Create a plane containing the target point, its normal and the current point
				Vector3f targetToPoint = point - targetPoint;
				Vector3f planeNormal = targetNormal.cross(targetToPoint).normalized();
				Hyperplane<float, 3> plane = Hyperplane<float, 3>(planeNormal, targetPoint);

				// Project current point's normal onto the plane and get the point's curvature
				Vector3f projectedNormal = plane.projection(normal);

				Vector3f pointDiff = targetPoint - point;
				double pointDistance = pointDiff.norm();
				Vector3f normalDiff = targetNormal - projectedNormal;
				double normalDistance = normalDiff.norm();

				if (pointDistance > 0)
					curvature += (normalDistance / pointDistance);
			}
			else
			{
				Vector3f projectedPoint = _bands[i].plane.projection(point);
				Vector3f projectedNormal = _bands[i].plane.projection(normal);

				Vector3f pointDiff = targetPoint - projectedPoint;
				double pointDistance = pointDiff.norm();
				Vector3f normalDiff = targetNormal - projectedNormal;
				double normalDistance = normalDiff.norm();

				if (pointDistance > 0)
					curvature += (normalDistance / pointDistance);
			}
		}
		curvature /= _bands[i].data->size();
		_curvatures.push_back(curvature);
	}
}

bool Helper::isNumber(const string &_str)
{
	bool number = true;
	for (size_t i = 0; i < _str.size(); i++)
	{
		number = number && isdigit(_str[i]);
	}
	return number;
}
