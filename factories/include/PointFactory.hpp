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
	static inline pcl::PointXYZ createPointXYZ(const float x_, const float y_, const float z_)
	{
		pcl::PointXYZ p;
		p.x = x_;
		p.y = y_;
		p.z = z_;
		return p;
	}

	// Creates a point with XYZ coordinates
	static inline pcl::PointXYZ createPointXYZ(const Eigen::Vector3f &data_)
	{
		pcl::PointXYZ p;
		p.x = data_.x();
		p.y = data_.y();
		p.z = data_.z();
		return p;
	}

	// Creates a point with XYZ coordinates and a label
	static inline pcl::PointXYZL createPointXYZL(const float x_, const float y_, const float z_, const int label_)
	{
		pcl::PointXYZL p;
		p.x = x_;
		p.y = y_;
		p.z = z_;
		p.label = label_;
		return p;
	}

	// Creates a point with XYZ coordinates and a RGB color definition associated
	static inline pcl::PointXYZRGB createPointXYZRGB(const float _x, const float _y, const float _z, const uint8_t r_, const uint8_t g_, const uint8_t b_)
	{
		pcl::PointXYZRGB p;
		p.x = _x;
		p.y = _y;
		p.z = _z;
		p.rgba = Utils::getColor(r_, g_, b_);
		return p;
	}

	// Creates a point with XYZ coordinates and a color definition associated
	static inline pcl::PointXYZRGB createPointXYZRGB(const float _x, const float _y, const float _z, const PointColor &color_)
	{
		pcl::PointXYZRGB p;
		p.x = _x;
		p.y = _y;
		p.z = _z;
		p.rgba = color_;
		return p;
	}

	// Creates a point with XYZ coordinates, a RGB color definition and a normal vector associated to each point
	static inline pcl::PointXYZRGBNormal createPointXYZRGBNormal(const float _x, const float _y, const float _z, const float _nx, const float _ny, const float _nz, const float _curvature, const uint8_t r_, const uint8_t g_, const uint8_t b_)
	{
		pcl::PointXYZRGBNormal p;
		p.x = _x;
		p.y = _y;
		p.z = _z;
		p.normal_x = _nx;
		p.normal_y = _ny;
		p.normal_z = _nz;
		p.curvature = _curvature;
		p.rgba = Utils::getColor(r_, g_, b_);
		return p;
	}

	// Creates a point with XYZ coordinates, a RGB color definition and a normal vector associated to each point
	static inline pcl::PointXYZRGBNormal createPointXYZRGBNormal(const float _x, const float _y, const float _z, const float _nx, const float _ny, const float _nz, const float _curvature, const PointColor &color_)
	{
		pcl::PointXYZRGBNormal p;
		p.x = _x;
		p.y = _y;
		p.z = _z;
		p.normal_x = _nx;
		p.normal_y = _ny;
		p.normal_z = _nz;
		p.curvature = _curvature;
		p.rgba = color_;
		return p;
	}

	// Creates a point with XYZ coordinates, a RGB color definition and a normal vector associated to each point
	static inline pcl::PointXYZRGBNormal createPointXYZRGBNormal(const float _x, const float _y, const float _z, const float _nx, const float _ny, const float _nz, const float _curvature, const uint32_t &color_)
	{
		pcl::PointXYZRGBNormal p;
		p.x = _x;
		p.y = _y;
		p.z = _z;
		p.normal_x = _nx;
		p.normal_y = _ny;
		p.normal_z = _nz;
		p.curvature = _curvature;
		p.rgba = color_;
		return p;
	}

	// Creates a point with XYZ coordinates and a normal vector associated to each point
	static inline pcl::PointNormal createPointNormal(const float x_, const float y_, const float z_, const float nx_, const float ny_, const float nz_, const float curvature_ = 0)
	{
		pcl::PointNormal p;
		p.x = x_;
		p.y = y_;
		p.z = z_;
		p.normal_x = nx_;
		p.normal_y = ny_;
		p.normal_z = nz_;
		p.curvature = curvature_;
		return p;
	}

private:
	PointFactory();
	~PointFactory();
};
