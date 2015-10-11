/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <vector>
#include "Extractor.h"
#include "Hist.h"
#include "../utils/ExecutionParams.h"

class Calculator
{
public:
	static std::vector<BandPtr> calculateDescriptor(pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const pcl::PointNormal &_point, const ExecutionParams &_params);

	static void calculateAngleHistograms(const std::vector<BandPtr> &_bands, std::vector<Hist> &_histograms, const bool _useProjection);
	static void calculateSequences(const std::vector<BandPtr> &_bands, const ExecutionParams &_params, const double _sequenceStep);

	template<class T>
	static inline double angle(const T &_vector1, const T &_vector2)
	{
		return atan2(_vector1.cross(_vector2).norm(), _vector1.dot(_vector2));
	}

	template<class T>
	static inline double signedAngle(const T &_vector1, const T &_vector2, const T &_normal)
	{
		double direction = _normal.dot(_vector1.cross(_vector2));

		// Check if the cross product is not zero
		if (fabs(direction) > 1E-7)
		{
			if (_normal.dot(_vector1.cross(_vector2)) < 0)
				return -atan2(_vector1.cross(_vector2).norm(), _vector1.dot(_vector2));
			else
				return atan2(_vector1.cross(_vector2).norm(), _vector1.dot(_vector2));
		}
		else
		{
			if (_vector1.dot(_vector2) >= 0)
				return 0;
			else
				return M_PI;

		}
	}

	static inline char getSequenceChar(const double _value, const double _step)
	{
		int index = _value / _step;
		if (index == 0)
			return '0';
		return index > 0 ? 'A' + index : 'a' - index;
	}

	static inline double calculateAngle(const Eigen::Vector3f &_vector1, const Eigen::Vector3f &_vector2, const Eigen::Hyperplane<float, 3> &_plane, const bool _useProjection)
	{
		Eigen::Vector3f v2 = _useProjection ? _plane.projection(_vector2).normalized() : _vector2;
		return signedAngle<Eigen::Vector3f>(_vector1, v2, (Eigen::Vector3f) _plane.normal());
	}

private:
	Calculator();
	~Calculator();
};

