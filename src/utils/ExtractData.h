/**
 * Author: rodrigo
 * 2015
 */
#pragma once
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <string>

class ExtractData
{
public:
	ExtractData()
	{
	}
	~ExtractData()
	{
	}

	static void extractSubCloud(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const std::string &_filename)
	{
		pcl::PointCloud<pcl::PointNormal>::Ptr extracted(new pcl::PointCloud<pcl::PointNormal>());

		std::vector<int> indices;
		indices.push_back(5465);
		indices.push_back(5541);
		indices.push_back(5616);
		indices.push_back(5617);
		indices.push_back(5619);
		indices.push_back(5691);
		indices.push_back(5692);
		indices.push_back(5693);
		indices.push_back(5769);
		indices.push_back(5912);

		indices.push_back(3343);
		indices.push_back(3426);
		indices.push_back(3512);
		indices.push_back(3515);
		indices.push_back(3594);
		indices.push_back(3676);
		indices.push_back(3681);
		indices.push_back(3759);
		indices.push_back(3837);
		indices.push_back(3839);

		indices.push_back(537);
		indices.push_back(659);
		indices.push_back(660);
		indices.push_back(784);
		indices.push_back(785);
		indices.push_back(786);
		indices.push_back(850);
		indices.push_back(911);
		indices.push_back(912);
		indices.push_back(1098);

		indices.push_back(22);
		indices.push_back(103);
		indices.push_back(104);
		indices.push_back(105);
		indices.push_back(107);
		indices.push_back(110);
		indices.push_back(154);
		indices.push_back(205);
		indices.push_back(210);
		indices.push_back(373);

		for (size_t i = 0; i < indices.size(); i++)
			extracted->push_back(_cloud->at(indices[i]));

		pcl::io::savePCDFileASCII(_filename, *extracted);
	}
};
