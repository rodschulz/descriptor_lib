/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <string>
#include <opencv2/core/core.hpp>
#include "Hist.hpp"
#include "Extractor.hpp"
#include "Metric.hpp"
#include "DescriptorParams.hpp"

class Writer
{
public:
	/**************************************************/
	static void writeHistogram(const std::string &filename_,
							   const std::string &histogramTitle_,
							   const std::vector<Hist> &histograms_,
							   const double binSize_,
							   const double lowerBound_ = -1,
							   const double upperBound_ = -1);

	/**************************************************/
	static void writeOuputData(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_,
							   const std::vector<BandPtr> &bands_,
							   const std::vector<Hist> &angleHistograms_,
							   const DescriptorParamsPtr &params_,
							   const int targetPoint_);

	/**************************************************/
	static void writePlotSSE(const std::string &filename_,
							 const std::string &plotTitle_,
							 const std::vector<double> &sse_);

	/**************************************************/
	static void writeClusteredCloud(const std::string &filename_,
									const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_,
									const cv::Mat &labels_);

	/**************************************************/
	static void writeDistanceMatrix(const std::string &filename_,
									const cv::Mat &items_,
									const cv::Mat &centers_,
									const cv::Mat &labels_,
									const MetricPtr &metric_);

	/**************************************************/
	static void writeClustersCenters(const std::string &filename_,
									 const cv::Mat &centers_,
									 const DescriptorParamsPtr &descriptorParams_,
									 const ClusteringParams &clusteringParams_,
									 const CloudSmoothingParams &smoothingParams_);

	/**************************************************/
	static void writeBoW(const std::string &filename_,
						 const cv::Mat &centers_,
						 const ClusteringParams &clusteringParams_,
						 const int nbands_,
						 const int nbins_,
						 const bool bidirectional_);

	/**************************************************/
	static void writeDescriptorsCache(const cv::Mat &descriptors_,
									  const std::string &cacheLocation_,
									  const std::string &cloudInputFilename_,
									  const double normalEstimationRadius_,
									  const DescriptorParamsPtr &descriptorParams_,
									  const CloudSmoothingParams &smoothingParams_);

	/**************************************************/
	static void saveCloudMatrix(const std::string &filename_,
								const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_);

private:
	// Constructor
	Writer();
	// Destructor
	~Writer();

	/**************************************************/
	static void writeMatrix(const std::string &filename_,
							const cv::Mat &matrix_,
							const std::vector<std::string> &metadata_ = std::vector<std::string>());

	/**************************************************/
	static void generateHistogramScript(const std::string &outputFolder_,
										const std::string &histogramTitle_,
										const int bandsNumber_,
										const double binSize_,
										const double lowerLimit_,
										const double upperLimit_);

	/**************************************************/
	static bool comparePairs(const std::pair<int, int> &item1_,
							 const std::pair<int, int> &item2_)
	{
		return item1_.second < item2_.second;
	}
};
