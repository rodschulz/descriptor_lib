/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <vector>
#include "ExecutionParams.hpp"
#include "Utils.hpp"
#include "Extractor.hpp"
#include "Hist.hpp"

// Type definition declaring the descriptor's structure
typedef std::vector<BandPtr> Descriptor;

class Calculator
{
public:
	// Calculates the descriptor over the given cloud, using the given params
	static Descriptor calculateDescriptor(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const DescriptorParams &_params, const int _targetPointIndex);

	// Calculates the descriptor over the given cloud, using the given params
	static Descriptor calculateDescriptor(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const DescriptorParams &_params, const pcl::PointNormal &_target);

	// Calculates the descriptor for each point in the given cloud and fills a matrix with the data
	static void calculateDescriptors(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const DescriptorParams &_params, cv::Mat &_descriptors);

	// Generates a vector holding the angular histograms for each band in the given array of bands (descriptor)
	static std::vector<Hist> generateAngleHistograms(const Descriptor &_descriptor, const bool _useProjection);

	// Calculates the sequence associated to each band in the descriptor
	static void fillSequences(Descriptor &_descriptor, const DescriptorParams &_params, const double _sequenceStep);

	// Returns a char to build a char sequence, according to the given value and step
	static inline char getSequenceChar(const double _value, const double _step)
	{
		int index = _value / _step;
		if (index == 0)
			return '0';

		return index > 0 ? 'A' + (index - 1) : 'a' - (index + 1);
	}

	// Calculates the angle between the two given vectors
	static inline double calculateAngle(const Eigen::Vector3f &_vector1, const Eigen::Vector3f &_vector2, const Eigen::Hyperplane<float, 3> &_plane, const bool _useProjection)
	{
		Eigen::Vector3f v2 = _useProjection ? _plane.projection(_vector2).normalized() : _vector2;
		return Utils::signedAngle<Eigen::Vector3f>(_vector1, v2, (Eigen::Vector3f) _plane.normal());
	}

private:
	Calculator();
	~Calculator();
};

