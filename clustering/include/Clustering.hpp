/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/core/core.hpp>
#include "ExecutionParams.hpp"
#include "Metric.hpp"


/**************************************************/
struct ClusteringResults
{
	cv::Mat labels;
	cv::Mat centers;
	std::vector<double> errorEvolution;

	/**************************************************/
	void prepare(const int ncluster_,
				 const int nitems_,
				 const int dim_)
	{
		centers = cv::Mat::zeros(ncluster_, dim_, CV_32FC1);
		labels = cv::Mat::zeros(nitems_, 1, CV_32SC1);
		errorEvolution.clear();
	}
};


/**************************************************/
class Clustering
{
public:
	/**************************************************/
	static void searchClusters(const cv::Mat &items_,
							   const ClusteringParams &params_,
							   ClusteringResults &results_);

	/**************************************************/
	static void generateElbowGraph(const cv::Mat &items_,
								   const ClusteringParams &params_);

	/**************************************************/
	static pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr generateClusterRepresentation(const pcl::PointCloud<pcl::PointNormal>::Ptr cloud_,
			const cv::Mat &labels_,
			const cv::Mat &centers_,
			const DescriptorParams &params_);


	/**************************************************/
	static cv::Mat generatePointDistanceMatrix(const cv::Mat &items_,
			const MetricPtr &metric_);

	/**************************************************/
	static cv::Mat generatePointDistanceMatrix2(const cv::Mat &items_,
			const MetricPtr &metric_);

	/**************************************************/
	static void generatePointDistanceMatrix3(cv::Mat &distanceMatrix_,
			const cv::Mat &items_,
			const MetricPtr &metric_);

private:
	Clustering();
	~Clustering();
};
