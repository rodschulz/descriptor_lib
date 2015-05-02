/**
 * Author: rodrigo
 * 2015
 */
#include "Helper.h"
#include <pcl/filters/filter.h>

Helper::Helper()
{
}

Helper::~Helper()
{
}

void Helper::removeNANs(PointCloud<PointXYZ>::Ptr &_cloud)
{
	std::vector<int> mapping;
	removeNaNFromPointCloud(*_cloud, *_cloud, mapping);
}

void Helper::createColorCloud(const PointCloud<PointXYZ>::Ptr &_cloud, PointCloud<PointXYZRGB>::Ptr &_coloredCloud, const uint8_t _r, const uint8_t _g, const uint8_t _b)
{
	_coloredCloud->points.clear();
	_coloredCloud->points.resize(_cloud->points.size());
	_coloredCloud->width = _cloud->points.size();
	_coloredCloud->height = 1;

	uint32_t color = ((uint32_t) _r << 16 | (uint32_t) _g << 8 | (uint32_t) _b);
	for (int i = 0; i < _cloud->width; i++)
	{
		_coloredCloud->points[i].x = _cloud->points[i].x;
		_coloredCloud->points[i].y = _cloud->points[i].y;
		_coloredCloud->points[i].z = _cloud->points[i].z;
		_coloredCloud->points[i].rgb = *reinterpret_cast<float*>(&color);
	}
}

float Helper::getColor(const uint8_t _r, const uint8_t _g, const uint8_t _b)
{
	uint32_t color = ((uint32_t) _r << 16 | (uint32_t) _g << 8 | (uint32_t) _b);
	float finalColor = *reinterpret_cast<float*>(&color);
	return finalColor;
}
