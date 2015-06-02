/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <pcl/point_types.h>

using namespace pcl;

class Factory
{
public:
	static PointXYZ makePointXYZ(const float _x, const float _y, const float _z);
	static PointXYZRGB makePointXYZRGB(const float _x, const float _y, const float _z, const float _rgb);
	static PointXYZRGB makePointXYZRGB(const float _x, const float _y, const float _z, const uint8_t _r, const uint8_t _g, const uint8_t _b);
	static PointNormal makePointNormal(const float _x, const float _y, const float _z, const float _nx, const float _ny, const float _nz, const float _curvature = 0);

private:
	Factory();
	~Factory();
};

