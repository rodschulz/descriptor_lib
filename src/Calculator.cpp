/**
 * Author: rodrigo
 * 2015
 */
#include "Calculator.h"
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <map>

using namespace boost::accumulators;

Calculator::Calculator()
{
}

Calculator::~Calculator()
{
}

void Calculator::calculateMeanCurvature(const vector<BandPtr> &_bands, vector<double> &_curvatures)
{
	_curvatures.clear();
	_curvatures.reserve(_bands.size());

	Vector3f targetPoint = _bands[0]->point.getVector3fMap();
	Vector3f targetNormal = _bands[0]->point.getNormalVector3fMap();

	for (size_t i = 0; i < _bands.size(); i++)
	{
		double curvature = 0;
		for (size_t j = 0; j < _bands[i]->data->size(); j++)
		{
			Vector3f point = _bands[i]->data->points[j].getVector3fMap();
			Vector3f normal = _bands[i]->data->points[j].getNormalVector3fMap();

			if (_bands[i]->isRadialBand)
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
				Vector3f projectedPoint = _bands[i]->plane.projection(point);
				Vector3f projectedNormal = _bands[i]->plane.projection(normal);

				Vector3f pointDiff = targetPoint - projectedPoint;
				double pointDistance = pointDiff.norm();
				Vector3f normalDiff = targetNormal - projectedNormal;
				double normalDistance = normalDiff.norm();

				if (pointDistance > 0)
					curvature += (normalDistance / pointDistance);
			}
		}
		curvature /= _bands[i]->data->size();
		_curvatures.push_back(curvature);
	}
}

void Calculator::calculateCurvatureHistograms(const vector<BandPtr> &_bands, vector<Hist> &_histograms)
{
	_histograms.clear();
	_histograms.reserve(_bands.size());

	Vector3f targetPoint = _bands[0]->point.getVector3fMap();
	Vector3f targetNormal = _bands[0]->point.getNormalVector3fMap();

	for (size_t i = 0; i < _bands.size(); i++)
	{
		Hist histogram;

		for (size_t j = 0; j < _bands[i]->data->size(); j++)
		{
			Vector3f point = _bands[i]->data->points[j].getVector3fMap();
			Vector3f normal = _bands[i]->data->points[j].getNormalVector3fMap();

			if (_bands[i]->isRadialBand)
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
				Vector3f projectedPoint = _bands[i]->plane.projection(point);
				Vector3f projectedNormal = _bands[i]->plane.projection(normal);

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

void Calculator::calculateAngleHistograms(const vector<BandPtr> &_bands, vector<Hist> &_histograms, const bool _useProjection)
{
	_histograms.clear();
	_histograms.reserve(_bands.size());

	Vector3f targetPoint = _bands[0]->point.getVector3fMap();
	Vector3f targetNormal = _bands[0]->point.getNormalVector3fMap();

	for (size_t i = 0; i < _bands.size(); i++)
	{
		BandPtr band = _bands[i];
		_histograms.push_back(Hist(ANGLE));

		for (size_t j = 0; j < band->data->size(); j++)
		{
			Vector3f point = band->data->points[j].getVector3fMap();
			Vector3f normal = band->data->points[j].getNormalVector3fMap();

			if (band->isRadialBand)
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
				double theta = calculateAngle(targetNormal, normal, band, _useProjection);
				_histograms.back().add(theta);
			}
		}
	}
}

void Calculator::calculateSequences(const vector<BandPtr> &_bands, const double _binSize, const double _sequenceStep, const bool _useProjection)
{
	for (size_t i = 0; i < _bands.size(); i++)
	{
		BandPtr band = _bands[i];

		Vector3f pointNormal = band->point.getNormalVector3fMap();
		Vector3f planeNormal = band->plane.normal();
		Vector3f n = planeNormal.cross(pointNormal).normalized();
		Hyperplane<float, 3> plane = Hyperplane<float, 3>(n, band->point.getVector3fMap());

		map<int, accumulator_set<double, features<tag::mean, tag::median> > > dataMap;
		for (size_t j = 0; j < band->data->size(); j++)
		{
			PointNormal p = band->data->at(j);
			double theta = calculateAngle(pointNormal, (Vector3f) p.getNormalVector3fMap(), band, _useProjection);
			int index = plane.signedDistance((Vector3f) p.getVector3fMap()) / _binSize;

			if (dataMap.find(index) == dataMap.end())
				dataMap[index] = accumulator_set<double, features<tag::mean, tag::median> >();

			dataMap[index](theta);
		}

		band->sequenceMean = band->sequenceMedian = "";
		for (map<int, accumulator_set<double, features<tag::mean, tag::median> > >::iterator it = dataMap.begin(); it != dataMap.end(); it++)
		{
			band->sequenceMean += getSequenceChar((double) mean(it->second), _sequenceStep);
			band->sequenceMedian += getSequenceChar((double) median(it->second), _sequenceStep);
		}
	}
}
