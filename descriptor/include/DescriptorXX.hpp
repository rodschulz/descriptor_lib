/**
 * Author: rodrigo
 * 2016
 */
#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/core/core.hpp>
#include "Params.hpp"


/**************************************************/
enum DescriptorType
{
	DESC_UNKNOWN,
	DESC_DCH,
	DESC_SHOT,
	DESC_USC,
	DESC_PHF,
	DESC_ROPS,
};


/**************************************************/
class DescriptorXX
{
public:
	DescriptorXX()
	{
		type = DESC_UNKNOWN;
	}
	virtual ~DescriptorXX() {}

	/**************************************************/
	virtual void computeDense(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_,
							  const DescriptorParamsPtr &params_,
							  cv::Mat &decriptors_) const = 0;

	/**************************************************/
	DescriptorType getType() const
	{
		return type;
	}
protected:
	DescriptorType type;
};


/**************************************************/
class DCH: public DescriptorXX
{
public:
	DCH()
	{
		type = DESC_DCH;
	}
	~DCH() {}

	/**************************************************/
	void computeDense(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_,
					  const DescriptorParamsPtr &params_,
					  cv::Mat &decriptors_) const;
};


/**************************************************/
class SHOT: public DescriptorXX
{
public:
	SHOT()
	{
		type = DESC_SHOT;
	}
	~SHOT() {}

	/**************************************************/
	void computeDense(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_,
					  const DescriptorParamsPtr &params_,
					  cv::Mat &decriptors_) const;
};


/**************************************************/