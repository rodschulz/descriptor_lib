/**
 * Author: rodrigo
 * 2015
 */
#include "PointFactory.hpp"

pcl::PointXYZ PointFactory::createPointXYZ(const float _x, const float _y, const float _z)
{
	pcl::PointXYZ p;
	p.x = _x;
	p.y = _y;
	p.z = _z;
	return p;
}

pcl::PointXYZ PointFactory::createPointXYZ(const Eigen::Vector3f &data_)
{
	pcl::PointXYZ p;
	p.x = data_.x();
	p.y = data_.y();
	p.z = data_.z();
	return p;
}

pcl::PointXYZRGB PointFactory::createPointXYZRGB(const float _x, const float _y, const float _z, const float _rgb)
{
	pcl::PointXYZRGB p;
	p.x = _x;
	p.y = _y;
	p.z = _z;
	p.rgb = _rgb;
	return p;
}

pcl::PointXYZRGB PointFactory::createPointXYZRGB(const float _x, const float _y, const float _z, const uint8_t _r, const uint8_t _g, const uint8_t _b)
{
	pcl::PointXYZRGB p;
	p.x = _x;
	p.y = _y;
	p.z = _z;
	p.rgb = Utils::getColor(_r, _g, _b);
	return p;
}

pcl::PointXYZRGB PointFactory::createPointXYZRGB(const float _x, const float _y, const float _z, const PointColor &_color)
{
	uint8_t r = (0xFF0000 & _color) >> 16;
	uint8_t g = (0x00FF00 & _color) >> 8;
	uint8_t b = 0x0000FF & _color;

	pcl::PointXYZRGB p;
	p.x = _x;
	p.y = _y;
	p.z = _z;
	p.rgb = Utils::getColor(r, g, b);
	return p;
}

pcl::PointXYZRGBNormal PointFactory::createPointXYZRGBNormal(const float _x, const float _y, const float _z, const float _nx, const float _ny, const float _nz, const float _curvature, const float _rgb)
{
	pcl::PointXYZRGBNormal p;
	p.x = _x;
	p.y = _y;
	p.z = _z;
	p.normal_x = _nx;
	p.normal_y = _ny;
	p.normal_z = _nz;
	p.curvature = _curvature;
	p.rgb = _rgb;
	return p;
}

pcl::PointXYZRGBNormal PointFactory::createPointXYZRGBNormal(const float _x, const float _y, const float _z, const float _nx, const float _ny, const float _nz, const float _curvature, const uint8_t _r, const uint8_t _g, const uint8_t _b)
{
	pcl::PointXYZRGBNormal p;
	p.x = _x;
	p.y = _y;
	p.z = _z;
	p.normal_x = _nx;
	p.normal_y = _ny;
	p.normal_z = _nz;
	p.curvature = _curvature;
	p.rgb = Utils::getColor(_r, _g, _b);
	return p;
}

pcl::PointXYZRGBNormal PointFactory::createPointXYZRGBNormal(const float _x, const float _y, const float _z, const float _nx, const float _ny, const float _nz, const float _curvature, const PointColor &_color)
{
	uint8_t r = (0xFF0000 & _color) >> 16;
	uint8_t g = (0x00FF00 & _color) >> 8;
	uint8_t b = 0x0000FF & _color;

	pcl::PointXYZRGBNormal p;
	p.x = _x;
	p.y = _y;
	p.z = _z;
	p.normal_x = _nx;
	p.normal_y = _ny;
	p.normal_z = _nz;
	p.curvature = _curvature;
	p.rgb = Utils::getColor(r, g, b);
	return p;
}

pcl::PointNormal PointFactory::createPointNormal(const float _x, const float _y, const float _z, const float _nx, const float _ny, const float _nz, const float _curvature)
{
	pcl::PointNormal p;
	p.x = _x;
	p.y = _y;
	p.z = _z;
	p.normal_x = _nx;
	p.normal_y = _ny;
	p.normal_z = _nz;
	p.curvature = _curvature;
	return p;
}
