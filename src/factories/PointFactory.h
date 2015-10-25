/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <pcl/point_types.h>

class PointFactory
{
public:
	static pcl::PointXYZ createPointXYZ(const float _x, const float _y, const float _z);
	static pcl::PointXYZRGB createPointXYZRGB(const float _x, const float _y, const float _z, const float _rgb);
	static pcl::PointXYZRGB createPointXYZRGB(const float _x, const float _y, const float _z, const uint8_t _r, const uint8_t _g, const uint8_t _b);
	static pcl::PointXYZRGBNormal createPointXYZRGBNormal(const float _x, const float _y, const float _z, const float _nx, const float _ny, const float _nz, const float _curvature, const uint8_t _r, const uint8_t _g, const uint8_t _b);
	static pcl::PointXYZRGBNormal createPointXYZRGBNormal(const float _x, const float _y, const float _z, const float _nx, const float _ny, const float _nz, const float _curvature, const float _rgb);
	static pcl::PointNormal createPointNormal(const float _x, const float _y, const float _z, const float _nx, const float _ny, const float _nz, const float _curvature = 0);

private:
	PointFactory();
	~PointFactory();
};
