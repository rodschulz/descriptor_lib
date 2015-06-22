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
				Vector3f targetToPoint = point - targetPoint;
				if (fabs(targetToPoint.norm()) < 1E-15)
					continue;

				// Create a plane containing the target point, its normal and the current point
				Vector3f planeNormal = targetNormal.cross(targetToPoint).normalized();
				Hyperplane<float, 3> plane = Hyperplane<float, 3>(planeNormal, targetPoint);

				_histograms.back().add(calculateAngle(targetNormal, normal, plane, _useProjection));
			}
			else
				_histograms.back().add(calculateAngle(targetNormal, normal, band->plane, _useProjection));
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
			double theta = calculateAngle(pointNormal, (Vector3f) p.getNormalVector3fMap(), band->plane, _useProjection);
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
