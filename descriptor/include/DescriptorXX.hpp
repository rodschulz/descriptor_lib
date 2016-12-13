/**
 * Author: rodrigo
 * 2016
 */
#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/core/core.hpp>
#include <boost/algorithm/string.hpp>
#include <yaml-cpp/yaml.h>
#include <string>


/**************************************************/
enum DescriptorType
{
	DESC_UNKNOWN,
	DESC_DCH,
	DESC_SHOT,
	DESC_USC,
	DESC_PFH,
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
	virtual void load(const YAML::Node &config_) = 0;
	virtual std::string toString() const = 0;

	/**************************************************/
	virtual void computeDense(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_,
							  const pcl::PointCloud<pcl::Normal>::Ptr &normals_,
							  cv::Mat &decriptors_) const = 0;

	/**************************************************/
	// virtual void computePoint(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_,
	// 						  const pcl::PointCloud<pcl::Normal>::Ptr &normals_,
	// 						  const int index_,
	// 						  cv::Mat &decriptor_) const = 0;

	// /**************************************************/
	// virtual void computePoint(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_,
	// 						  const pcl::PointCloud<pcl::Normal>::Ptr &normals_,
	// 						  const pcl::PointXYZ &point_,
	// 						  cv::Mat &decriptor_) const = 0;

	/**************************************************/
	DescriptorType getType() const
	{
		return type;
	}

	/**************************************************/
	static DescriptorType toType(const std::string &str_)
	{
		if (boost::iequals(str_, "dch"))
			return DESC_DCH;

		else if (boost::iequals(str_, "shot"))
			return DESC_SHOT;

		else if (boost::iequals(str_, "usc"))
			return DESC_USC;

		else if (boost::iequals(str_, "pfh"))
			return DESC_PFH;

		else if (boost::iequals(str_, "rops"))
			return DESC_ROPS;

		else
			return DESC_UNKNOWN;
	}

protected:
	DescriptorType type;
};

typedef boost::shared_ptr<DescriptorXX> DescriptorPtr;
