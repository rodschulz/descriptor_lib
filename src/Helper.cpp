/**
 * Author: rodrigo
 * 2015
 */
#include "Helper.h"
#include <pcl/filters/filter.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d_omp.h>
#include <ctype.h>
#include "Factory.h"
#include "CloudFactory.h"

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

PointCloud<Normal>::Ptr Helper::getNormals(const PointCloud<PointXYZ>::Ptr &_cloud, const double _searchRadius)
{
	PointCloud<Normal>::Ptr normals(new PointCloud<Normal>());

	search::KdTree<PointXYZ>::Ptr kdtree(new search::KdTree<PointXYZ>);
	NormalEstimation<PointXYZ, Normal> normalEstimation;
	normalEstimation.setInputCloud(_cloud);

	if (_searchRadius > 0)
		normalEstimation.setRadiusSearch(_searchRadius);
	else
		normalEstimation.setKSearch(10);

	normalEstimation.setSearchMethod(kdtree);
	normalEstimation.compute(*normals);

	return normals;
}

bool Helper::getCloudAndNormals(PointCloud<PointNormal>::Ptr &_cloud, const ExecutionParams &_params)
{
	bool loadOk = true;

	// Get cartesian data
	PointCloud<PointXYZ>::Ptr cloudXYZ(new PointCloud<PointXYZ>());
	if (!_params.useSynthetic)
	{
		if (io::loadPCDFile<PointXYZ>(_params.inputLocation, *cloudXYZ) != 0)
		{
			cout << "ERROR: Can't read file from disk (" << _params.inputLocation << ")\n";
			loadOk = false;
		}
	}
	else
	{
		switch (_params.synCloudType)
		{
			case CUBE:
				CloudFactory::generateCube(0.3, Factory::makePointXYZ(0.3, 0.3, 0.3), cloudXYZ);
				break;

			case CYLINDER:
				CloudFactory::generateCylinder(0.2, 0.5, Factory::makePointXYZ(0.4, 0.4, 0.4), cloudXYZ);
				break;

			case SPHERE:
				CloudFactory::generateSphere(0.2, Factory::makePointXYZ(0.2, 0.2, 0.2), cloudXYZ);
				break;

			default:
				cloudXYZ->clear();
				cout << "WARNING, wrong cloud generation parameters\n";
		}
	}

	// Estimate normals
	if (loadOk)
	{
		Helper::removeNANs(cloudXYZ);
		PointCloud<Normal>::Ptr normals = Helper::getNormals(cloudXYZ, _params.normalEstimationRadius);

		_cloud->clear();
		concatenateFields(*cloudXYZ, *normals, *_cloud);
	}

	return loadOk;
}

PointCloud<PointXYZRGBNormal>::Ptr Helper::createColorCloud(const PointCloud<PointNormal>::Ptr &_cloud, uint32_t _color)
{
	PointCloud<PointXYZRGBNormal>::Ptr coloredCloud(new PointCloud<PointXYZRGBNormal>());
	coloredCloud->reserve(_cloud->size());

	float color = *reinterpret_cast<float*>(&_color);
	for (int i = 0; i < _cloud->width; i++)
	{
		PointNormal p = _cloud->points[i];
		coloredCloud->push_back(Factory::makePointXYZRGBNormal(p.x, p.y, p.z, p.normal_x, p.normal_y, p.normal_z, p.curvature, color));
	}

	return coloredCloud;
}

PointCloud<PointXYZRGBNormal>::Ptr Helper::createColorCloud(const PointCloud<PointNormal>::Ptr &_cloud, uint8_t _r, uint8_t _g, uint8_t _b)
{
	PointCloud<PointXYZRGBNormal>::Ptr coloredCloud(new PointCloud<PointXYZRGBNormal>());
	coloredCloud->reserve(_cloud->size());

	for (int i = 0; i < _cloud->width; i++)
	{
		PointNormal p = _cloud->points[i];
		coloredCloud->push_back(Factory::makePointXYZRGBNormal(p.x, p.y, p.z, p.normal_x, p.normal_y, p.normal_z, p.curvature, _r, _g, _b));
	}

	return coloredCloud;
}

float Helper::getColor(const uint8_t _r, const uint8_t _g, const uint8_t _b)
{
	uint32_t color = ((uint32_t) _r << 16 | (uint32_t) _g << 8 | (uint32_t) _b);
	float finalColor = *reinterpret_cast<float*>(&color);
	return finalColor;
}

bool Helper::isNumber(const string &_str)
{
	bool number = true;
	for (size_t i = 0; i < _str.size(); i++)
	{
		number = number && isdigit(_str[i]);
	}
	return number;
}

unsigned int Helper::getColor(const int _index)
{
	static uint32_t palette[12] =
	{ 0xa6cee3, 0x1f78b4, 0xb2df8a, 0x33a02c, 0xfb9a99, 0xe31a1c, 0xfdbf6f, 0xff7f00, 0xcab2d6, 0x6a3d9a, 0xffff99, 0xb15928 };

	return palette[_index % 12];
}
