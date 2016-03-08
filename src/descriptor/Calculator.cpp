/**
 * Author: rodrigo
 * 2015
 */
#include "Calculator.hpp"
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <boost/accumulators/statistics/count.hpp>
#include <map>

using namespace boost::accumulators;

Descriptor Calculator::calculateDescriptor(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const ExecutionParams &_params)
{
	pcl::PointNormal target = _cloud->at(_params.targetPoint);
	return calculateDescriptor(_cloud, _params, target);
}

Descriptor Calculator::calculateDescriptor(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const ExecutionParams &_params, const pcl::PointNormal &_target)
{
	// Get target point and surface patch
	pcl::PointCloud<pcl::PointNormal>::Ptr patch = Extractor::getNeighbors(_cloud, _target, _params.patchSize);

	// Extract bands
	std::vector<BandPtr> bands = Extractor::getBands(patch, _target, _params);
	Calculator::fillSequences(bands, _params, M_PI / 18);

	return bands;
}

void Calculator::calculateDescriptors(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const ExecutionParams &_params, cv::Mat &_descriptors)
{
	int sequenceSize = _params.getSequenceLength();

	// Resize the matrix in case it doesn't match the required dimensions
	int rows = _cloud->size();
	int cols = sequenceSize * _params.bandNumber;
	if (_descriptors.rows != rows || _descriptors.cols != cols)
		_descriptors = cv::Mat::zeros(rows, cols, CV_32FC1);

	// Extract the descriptors
	for (size_t i = 0; i < _cloud->size(); i++)
	{
		std::vector<BandPtr> bands = Calculator::calculateDescriptor(_cloud, _params, _cloud->points[i]);

		for (size_t j = 0; j < bands.size(); j++)
			memcpy(&_descriptors.at<float>(i, j * sequenceSize), &bands[j]->sequenceVector[0], sizeof(float) * sequenceSize);
	}
}

std::vector<Hist> Calculator::generateAngleHistograms(const Descriptor &_descriptor, const bool _useProjection)
{
	std::vector<Hist> histograms = std::vector<Hist>();
	histograms.reserve(_descriptor.size());

	Eigen::Vector3f targetNormal = _descriptor[0]->point.getNormalVector3fMap();
	for (size_t i = 0; i < _descriptor.size(); i++)
	{
		BandPtr band = _descriptor[i];
		histograms.push_back(Hist(ANGLE));

		for (size_t j = 0; j < band->data->size(); j++)
		{
			Eigen::Vector3f normal = band->data->points[j].getNormalVector3fMap();
			histograms.back().add(calculateAngle(targetNormal, normal, band->plane, _useProjection));
		}
	}

	return histograms;
}

void Calculator::fillSequences(Descriptor &_descriptor, const ExecutionParams &_params, const double _sequenceStep)
{
	double binSize = _params.sequenceBin;
	int binsNumber = _params.getSequenceLength();

	for (size_t i = 0; i < _descriptor.size(); i++)
	{
		BandPtr band = _descriptor[i];

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
				band->sequenceVector.push_back(5);
			}
		}
	}
}
