/**
 * Author: rodrigo
 * 2015
 */
#include "Calculator.h"
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <boost/accumulators/statistics/count.hpp>
#include <map>

using namespace boost::accumulators;

Calculator::Calculator()
{
}

Calculator::~Calculator()
{
}

std::vector<BandPtr> Calculator::calculateDescriptor(pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const pcl::PointNormal &_target, const ExecutionParams &_params)
{
	// Get target point and surface patch
	pcl::PointCloud<pcl::PointNormal>::Ptr patch = Extractor::getNeighbors(_cloud, _target, _params.patchSize);

	// Extract bands
	std::vector<BandPtr> bands = Extractor::getBands(patch, _target, _params);
	Calculator::calculateSequences(bands, _params, M_PI / 18);

	return bands;
}

void Calculator::calculateAngleHistograms(const std::vector<BandPtr> &_bands, std::vector<Hist> &_histograms, const bool _useProjection)
{
	_histograms.clear();
	_histograms.reserve(_bands.size());

	Eigen::Vector3f targetPoint = _bands[0]->point.getVector3fMap();
	Eigen::Vector3f targetNormal = _bands[0]->point.getNormalVector3fMap();

	for (size_t i = 0; i < _bands.size(); i++)
	{
		BandPtr band = _bands[i];
		_histograms.push_back(Hist(ANGLE));

		for (size_t j = 0; j < band->data->size(); j++)
		{
			Eigen::Vector3f point = band->data->points[j].getVector3fMap();
			Eigen::Vector3f normal = band->data->points[j].getNormalVector3fMap();
			_histograms.back().add(calculateAngle(targetNormal, normal, band->plane, _useProjection));
		}
	}
}

void Calculator::calculateSequences(const std::vector<BandPtr> &_bands, const ExecutionParams &_params, const double _sequenceStep)
{
	double binSize = _params.sequenceBin;
	int binsNumber = calculateSequenceLength(_params);

	for (size_t i = 0; i < _bands.size(); i++)
	{
		BandPtr band = _bands[i];

		Eigen::Vector3f pointNormal = band->point.getNormalVector3fMap();
		Eigen::Vector3f planeNormal = band->plane.normal();
		Eigen::Vector3f n = planeNormal.cross(pointNormal).normalized();
		Eigen::Hyperplane<float, 3> plane = Eigen::Hyperplane<float, 3>(n, band->point.getVector3fMap());

		std::map<int, accumulator_set<double, features<tag::mean, tag::median, tag::min> > > dataMap;
		for (size_t j = 0; j < band->data->size(); j++)
		{
			pcl::PointNormal p = band->data->at(j);
			double theta = calculateAngle(pointNormal, (Eigen::Vector3f) p.getNormalVector3fMap(), band->plane, _params.useProjection);
			int index = plane.signedDistance((Eigen::Vector3f) p.getVector3fMap()) / binSize;

			if (dataMap.find(index) == dataMap.end())
				dataMap[index] = accumulator_set<double, features<tag::mean, tag::median, tag::min> >();

			dataMap[index](theta);
		}

		band->sequenceString = "";
		for (int j = 0; j < binsNumber; j++)
		{
			if (dataMap.find(j) != dataMap.end())
			{
				float value = _params.sequenceStat == STAT_MEAN ? (float) mean(dataMap[j]) : (float) median(dataMap[j]);
				band->sequenceString += getSequenceChar(value, _sequenceStep);
				band->sequenceVector.push_back(value);
			}
			else
			{
				band->sequenceString += '-';
				band->sequenceVector.push_back(-M_PI);
			}
		}
	}
}
