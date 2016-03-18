/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <pcl/point_types.h>

// Colors defined to be used for points
typedef enum PointColor
{
	COLOR_BLACK	= 0x000000,
	COLOR_WHITE	= 0xFFFFFF,
	COLOR_RED	= 0xFF0000,
	COLOR_GREEN	= 0x00FF00,
	COLOR_BLUE	= 0x0000FF,
	COLOR_CYAN	= 0X00FFFF,
	COLOR_MAGENTA	= 0xFF00FF,
	COLOR_YELLOW	= 0xFFFF00,
} PointColor;

// Point factory class
class PointFactory
{
public:
	// Creates a point with XYZ coordinates
	static pcl::PointXYZ createPointXYZ(const float _x, const float _y, const float _z);

	// Creates a point with XYZ coordinates and a RGB color definition associated
	static pcl::PointXYZRGB createPointXYZRGB(const float _x, const float _y, const float _z, const float _rgb);

	// Creates a point with XYZ coordinates and a RGB color definition associated
	static pcl::PointXYZRGB createPointXYZRGB(const float _x, const float _y, const float _z, const uint8_t _r, const uint8_t _g, const uint8_t _b);

	// Creates a point with XYZ coordinates and a color definition associated
	static pcl::PointXYZRGB createPointXYZRGB(const float _x, const float _y, const float _z, const PointColor &_color);

	// Creates a point with XYZ coordinates, a RGB color definition and a normal vector associated to each point
	static pcl::PointXYZRGBNormal createPointXYZRGBNormal(const float _x, const float _y, const float _z, const float _nx, const float _ny, const float _nz, const float _curvature, const uint8_t _r, const uint8_t _g, const uint8_t _b);

	// Creates a point with XYZ coordinates, a RGB color definition and a normal vector associated to each point
	static pcl::PointXYZRGBNormal createPointXYZRGBNormal(const float _x, const float _y, const float _z, const float _nx, const float _ny, const float _nz, const float _curvature, const float _rgb);

	// Creates a point with XYZ coordinates and a normal vector associated to each point
	static pcl::PointNormal createPointNormal(const float _x, const float _y, const float _z, const float _nx, const float _ny, const float _nz, const float _curvature = 0);

private:
	PointFactory();
	~PointFactory();
};
