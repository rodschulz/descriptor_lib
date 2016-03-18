/**
 * Author: rodrigo
 * 2015
 */
#include "CloudFactory.hpp"
#include "PointFactory.hpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CloudFactory::createCube(const double _size, const pcl::PointXYZ &_center)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

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
			cloud->push_back(PointFactory::createPointXYZ(minX, y, z));
			cloud->push_back(PointFactory::createPointXYZ(maxX, y, z));
		}
	}

	// Generate faces fixed in Y axis
	for (double x = minX; x <= maxX; x += step)
	{
		for (double z = minZ; z <= maxZ; z += step)
		{
			cloud->push_back(PointFactory::createPointXYZ(x, minY, z));
			cloud->push_back(PointFactory::createPointXYZ(x, maxY, z));
		}
	}

	// Generate faces fixed in Z axis
	for (double x = minX; x <= maxX; x += step)
	{
		for (double y = minY; y <= maxY; y += step)
		{
			cloud->push_back(PointFactory::createPointXYZ(x, y, minZ));
			cloud->push_back(PointFactory::createPointXYZ(x, y, maxZ));
		}
	}

	return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CloudFactory::createCylinder(const double _radius, const double _height, const pcl::PointXYZ &_center)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

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
			cloud->push_back(PointFactory::createPointXYZ(r * cos(angle) + _center.x, r * sin(angle) + _center.y, minZ));
			cloud->push_back(PointFactory::createPointXYZ(r * cos(angle) + _center.x, r * sin(angle) + _center.y, maxZ));
		}
	}

	// Generate cylinder side
	for (double z = minZ; z <= maxZ; z += stepZ)
	{
		for (double angle = 0; angle < 2 * M_PI; angle += angularStep)
		{
			cloud->push_back(PointFactory::createPointXYZ(_radius * cos(angle) + _center.x, _radius * sin(angle) + _center.y, z));
		}
	}

	return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CloudFactory::createSphere(const double _radius, const pcl::PointXYZ &_center)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

	double angularStep = 2 * M_PI * 0.005;
	for (double theta = 0; theta < 2 * M_PI; theta += angularStep)
	{
		for (double phi = 0; phi < 2 * M_PI; phi += angularStep)
		{
			cloud->push_back(PointFactory::createPointXYZ(_radius * sin(theta) * cos(phi) + _center.x, _radius * sin(theta) * sin(phi) + _center.y, _radius * cos(theta) + _center.z));
		}
	}

	return cloud;
}

pcl::PointCloud<pcl::PointNormal>::Ptr CloudFactory::createHorizontalPlane(const float _minX, const float _maxX, const float _minY, const float _maxY, const float _z, const int _npoints)
{
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>());
	int N = sqrt(_npoints);

	// Set the steps for aprox N points
	float stepX = (_maxX - _minX) / N;
	float stepY = (_maxY - _minY) / N;

	for (float dx = _minX; dx <= _maxX; dx += stepX)
		for (float dy = _minY; dy <= _maxY; dy += stepY)
			cloud->push_back(PointFactory::createPointNormal(dx, dy, _z, 0, 0, 1));

	return cloud;
}

pcl::PointCloud<pcl::PointNormal>::Ptr CloudFactory::createSphereSection(const float _azimuth, const float _radius, const Eigen::Vector3f &_center, const int _npoints)
{
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>());

	int N = sqrt(_npoints);
	float azimuth = _azimuth <= 2 * M_PI ? _azimuth : 2 * M_PI;

	float stepPolar = M_PI / N;
	float stepAzimuth = azimuth / N;

	for (float theta = 0; theta < azimuth; theta += stepAzimuth) // Theta belongs to [0, 2PI]
	{
		for (float phi = 0; phi < M_PI; phi += stepPolar) // Phi belongs to [0, PI]
		{
			Eigen::Vector3f point = Eigen::Vector3f(_radius * sin(theta) * cos(phi) + _center.x(),
								_radius * sin(theta) * sin(phi) + _center.y(),
								_radius * cos(theta) + _center.z());
			Eigen::Vector3f normal = (point - _center).normalized();
			cloud->push_back(PointFactory::createPointNormal(point.x(), point.y(), point.z(), normal.x(), normal.y(), normal.z()));
		}
	}

	return cloud;
}

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr CloudFactory::createColorCloud(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, uint32_t _color)
{
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr coloredCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
	coloredCloud->reserve(_cloud->size());

	float color = *reinterpret_cast<float*>(&_color);
	for (unsigned int i = 0; i < _cloud->width; i++)
	{
		pcl::PointNormal p = _cloud->points[i];
		coloredCloud->push_back(PointFactory::createPointXYZRGBNormal(p.x, p.y, p.z, p.normal_x, p.normal_y, p.normal_z, p.curvature, color));
	}

	return coloredCloud;
}

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr CloudFactory::createColorCloud(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, uint8_t _r, uint8_t _g, uint8_t _b)
{
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr coloredCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
	coloredCloud->reserve(_cloud->size());

	for (unsigned int i = 0; i < _cloud->width; i++)
	{
		pcl::PointNormal p = _cloud->points[i];
		coloredCloud->push_back(PointFactory::createPointXYZRGBNormal(p.x, p.y, p.z, p.normal_x, p.normal_y, p.normal_z, p.curvature, _r, _g, _b));
	}

	return coloredCloud;
}
