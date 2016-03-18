/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <pcl/point_types.h>

// Colors defined to be used for points
typedef enum PointColor
{
	COLOR_WHITE = 0xFFFFFF,
	COLOR_LIGHT_GRAY = 0xD3D3D3,
	COLOR_GRAY = 0x808080,
	COLOR_DARK_GRAY = 0xA9A9A9,
	COLOR_BLACK = 0x000000,
	COLOR_BROWN = 0x8B4513,
	COLOR_HOT_PINK = 0xFF69B4,
	COLOR_PINK = 0xFFC0CB,
	COLOR_MAGENTA = 0xFF00FF,
	COLOR_VIOLET = 0x9400D3,
	COLOR_SKY_BLUE = 0x87CEFA,
	COLOR_BLUE = 0x0000FF,
	COLOR_TURQUOISE = 0x40E0D0,
	COLOR_CYAN = 0X00FFFF,
	COLOR_LIGHT_GREEN = 0x98FB98,
	COLOR_GREEN = 0x00FF00,
	COLOR_DARK_GREEN = 0x008000,
	COLOR_YELLOW = 0xFFFF00,
	COLOR_ORANGE = 0xFFA500,
	COLOR_RED = 0xFF0000,
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
	static pcl::PointXYZRGBNormal createPointXYZRGBNormal(const float _x, const float _y, const float _z, const float _nx, const float _ny, const float _nz, const float _curvature, const float _rgb);

	// Creates a point with XYZ coordinates, a RGB color definition and a normal vector associated to each point
	static pcl::PointXYZRGBNormal createPointXYZRGBNormal(const float _x, const float _y, const float _z, const float _nx, const float _ny, const float _nz, const float _curvature, const uint8_t _r, const uint8_t _g, const uint8_t _b);

	// Creates a point with XYZ coordinates, a RGB color definition and a normal vector associated to each point
	static pcl::PointXYZRGBNormal createPointXYZRGBNormal(const float _x, const float _y, const float _z, const float _nx, const float _ny, const float _nz, const float _curvature, const PointColor &_color);

	// Creates a point with XYZ coordinates and a normal vector associated to each point
	static pcl::PointNormal createPointNormal(const float _x, const float _y, const float _z, const float _nx, const float _ny, const float _nz, const float _curvature = 0);

private:
	PointFactory();
	~PointFactory();
};
