/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include "Hist.h"
#include "Extractor.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

using namespace std;
using namespace pcl;

// Sign function
template<typename T> int sign(T val)
{
	return (T(0) < val) - (val < T(0));
}

class Helper
{
public:
	static void removeNANs(PointCloud<PointXYZ>::Ptr &_cloud);
	static void createColorCloud(const PointCloud<PointXYZ>::Ptr &_cloud, PointCloud<PointXYZRGB>::Ptr &_coloredCloud, const uint8_t _r, const uint8_t _g, const uint8_t _b);
	static float getColor(const uint8_t _r, const uint8_t _g, const uint8_t _b);
	static void calculateAngleHistograms(const vector<Band> &_bands, const PointXYZ &_point, vector<Hist> &_histograms);
	static void calculateCurvatureHistograms(const vector<Band> &_bands, const PointXYZ &_point, vector<Hist> &_histograms);
	static void calculateMeanCurvature(const vector<Band> &_bands, const PointXYZ &_point, vector<double> &_curvatures);
	static bool isNumber(const string &_str);

	 template <class T>
	 static inline double angleBetween(const T &_vector1, const T &_vector2)
	{
		return atan2(_vector1.cross(_vector2).norm(), _vector1.dot(_vector2));
	}

private:
	Helper();
	~Helper();
};
