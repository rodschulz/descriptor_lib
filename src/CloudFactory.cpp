/**
 * Author: rodrigo
 * 2015
 */
#include "CloudFactory.h"
#include "Factory.h"

CloudFactory::CloudFactory()
{
}

CloudFactory::~CloudFactory()
{
}

void CloudFactory::generateCube(const double _size, const PointXYZ &_center, PointCloud<PointXYZ>::Ptr &_cloud)
{
	_cloud->clear();

	double minX = _center.x - _size * 0.5;
	double minY = _center.y - _size * 0.5;
	double minZ = _center.z - _size * 0.5;

	double maxX = _center.x + _size * 0.5;
	double maxY = _center.y + _size * 0.5;
	double maxZ = _center.z + _size * 0.5;

	double step = _size * 0.01;

	// Generate faces fixed in X axis
	for (double y = minY; y <= maxY; y += step)
	{
		for (double z = minZ; z <= maxZ; z += step)
		{
			_cloud->push_back(Factory::makePointXYZ(minX, y, z));
			_cloud->push_back(Factory::makePointXYZ(maxX, y, z));
		}
	}

	// Generate faces fixed in Y axis
	for (double x = minX; x <= maxX; x += step)
	{
		for (double z = minZ; z <= maxZ; z += step)
		{
			_cloud->push_back(Factory::makePointXYZ(x, minY, z));
			_cloud->push_back(Factory::makePointXYZ(x, maxY, z));
		}
	}

	// Generate faces fixed in Z axis
	for (double x = minX; x <= maxX; x += step)
	{
		for (double y = minY; y <= maxY; y += step)
		{
			_cloud->push_back(Factory::makePointXYZ(x, y, minZ));
			_cloud->push_back(Factory::makePointXYZ(x, y, maxZ));
		}
	}
}

void CloudFactory::generateCylinder(const double _radius, const double _height, const PointXYZ &_center, PointCloud<PointXYZ>::Ptr &_cloud)
{
	_cloud->clear();

	double minZ = _center.z - _height * 0.5;
	double maxZ = _center.z + _height * 0.5;
	double stepZ = _height * 0.01;

	double angularStep = 2 * M_PI * 0.005;
	double radialStep = _radius * 0.02;

	// Generate cylinder top and bottom
	for (double angle = 0; angle < 2 * M_PI; angle += angularStep)
	{
		for (double r = _radius; r > 0; r -= radialStep)
		{
			_cloud->push_back(Factory::makePointXYZ(r * cos(angle), r * sin(angle), minZ));
			_cloud->push_back(Factory::makePointXYZ(r * cos(angle), r * sin(angle), maxZ));
		}
	}

	// Generate cylinder side
	for (double z = minZ; z <= maxZ; z += stepZ)
	{
		for (double angle = 0; angle < 2 * M_PI; angle += angularStep)
		{
			_cloud->push_back(Factory::makePointXYZ(_radius * cos(angle), _radius * sin(angle), z));
		}
	}
}

void CloudFactory::generateSphere(const double _radius, const PointXYZ &_center, PointCloud<PointXYZ>::Ptr &_cloud)
{
	_cloud->clear();

	double angularStep = 2 * M_PI * 0.005;
	for (double theta = 0; theta < 2 * M_PI; theta += angularStep)
	{
		for (double phi = 0; phi < 2 * M_PI; phi += angularStep)
		{
			_cloud->push_back(Factory::makePointXYZ(_radius * sin(theta) * cos(phi), _radius * sin(theta) * sin(phi), _radius * cos(theta)));
		}
	}
}
