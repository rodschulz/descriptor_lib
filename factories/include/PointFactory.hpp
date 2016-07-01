/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <pcl/point_types.h>
#include "Utils.hpp"

// Point factory class
class PointFactory
{
public:
	// Creates a point with XYZ coordinates
	static pcl::PointXYZ createPointXYZ(const float _x, const float _y, const float _z);

	// Creates a point with XYZ coordinates
	static pcl::PointXYZ createPointXYZ(const Eigen::Vector3f &data_);

	// Creates a point with XYZ coordinates and a RGB color definition associated
	static pcl::PointXYZRGB createPointXYZRGB(const float _x, const float _y, const float _z, const uint8_t _r, const uint8_t _g, const uint8_t _b);

	// Creates a point with XYZ coordinates and a color definition associated
	static pcl::PointXYZRGB createPointXYZRGB(const float _x, const float _y, const float _z, const PointColor &_color);

	// Creates a point with XYZ coordinates, a RGB color definition and a normal vector associated to each point
	static pcl::PointXYZRGBNormal createPointXYZRGBNormal(const float _x, const float _y, const float _z, const float _nx, const float _ny, const float _nz, const float _curvature, const uint8_t _r, const uint8_t _g, const uint8_t _b);

	// Creates a point with XYZ coordinates, a RGB color definition and a normal vector associated to each point
	static pcl::PointXYZRGBNormal createPointXYZRGBNormal(const float _x, const float _y, const float _z, const float _nx, const float _ny, const float _nz, const float _curvature, const PointColor &_color);

	// Creates a point with XYZ coordinates, a RGB color definition and a normal vector associated to each point
	static pcl::PointXYZRGBNormal createPointXYZRGBNormal(const float _x, const float _y, const float _z, const float _nx, const float _ny, const float _nz, const float _curvature, const uint32_t &_color);

	// Creates a point with XYZ coordinates and a normal vector associated to each point
	static pcl::PointNormal createPointNormal(const float _x, const float _y, const float _z, const float _nx, const float _ny, const float _nz, const float _curvature = 0);

private:
	PointFactory();
	~PointFactory();
};
