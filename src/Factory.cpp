/**
 * Author: rodrigo
 * 2015
 */
#include "Factory.h"

Factory::Factory()
{
}

Factory::~Factory()
{
}

PointXYZ Factory::makePointXYZ(const float _x, const float _y, const float _z)
{
	PointXYZ p;
	p.x = _x;
	p.y = _y;
	p.z = _z;
	return p;
}

PointXYZRGB Factory::makePointXYZRGB(const float _x, const float _y, const float _z, const float _rgb)
{
	PointXYZRGB p;
	p.x = _x;
	p.y = _y;
	p.z = _z;
	p.rgb = _rgb;
	return p;
}

PointXYZRGB Factory::makePointXYZRGB(const float _x, const float _y, const float _z, const uint8_t _r, const uint8_t _g, const uint8_t _b)
{
	uint32_t color = ((uint32_t) _r << 16 | (uint32_t) _g << 8 | (uint32_t) _b);

	PointXYZRGB p;
	p.x = _x;
	p.y = _y;
	p.z = _z;
	p.rgb = *reinterpret_cast<float*>(&color);
	return p;
}

PointNormal Factory::makePointNormal(const float _x, const float _y, const float _z, const float _nx, const float _ny, const float _nz, const float _curvature)
{
	PointNormal p;
	p.x = _x;
	p.y = _y;
	p.z = _z;
	p.normal_x = _nx;
	p.normal_y = _ny;
	p.normal_z = _nz;
	p.curvature = _curvature;
	return p;
}
