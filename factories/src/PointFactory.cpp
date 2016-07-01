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

pcl::PointXYZRGB PointFactory::createPointXYZRGB(const float _x, const float _y, const float _z, const uint8_t _r, const uint8_t _g, const uint8_t _b)
{
	pcl::PointXYZRGB p;
	p.x = _x;
	p.y = _y;
	p.z = _z;
	p.rgba = Utils::getColor(_r, _g, _b);
	return p;
}

pcl::PointXYZRGB PointFactory::createPointXYZRGB(const float _x, const float _y, const float _z, const PointColor &_color)
{
	pcl::PointXYZRGB p;
	p.x = _x;
	p.y = _y;
	p.z = _z;
	p.rgba = _color;
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
	p.rgba = Utils::getColor(_r, _g, _b);
	return p;
}

pcl::PointXYZRGBNormal PointFactory::createPointXYZRGBNormal(const float _x, const float _y, const float _z, const float _nx, const float _ny, const float _nz, const float _curvature, const PointColor &_color)
{
	pcl::PointXYZRGBNormal p;
	p.x = _x;
	p.y = _y;
	p.z = _z;
	p.normal_x = _nx;
	p.normal_y = _ny;
	p.normal_z = _nz;
	p.curvature = _curvature;
	p.rgba = _color;
	return p;
}

pcl::PointXYZRGBNormal PointFactory::createPointXYZRGBNormal(const float _x, const float _y, const float _z, const float _nx, const float _ny, const float _nz, const float _curvature, const uint32_t &_color)
{
	pcl::PointXYZRGBNormal p;
	p.x = _x;
	p.y = _y;
	p.z = _z;
	p.normal_x = _nx;
	p.normal_y = _ny;
	p.normal_z = _nz;
	p.curvature = _curvature;
	p.rgba = _color;
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
