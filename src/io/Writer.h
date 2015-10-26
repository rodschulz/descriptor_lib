/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <string>
#include "../descriptor/Hist.h"
#include "../descriptor/Extractor.h"

#define OUTPUT_FOLDER		"./output/"

class Writer
{
public:
	static void writeHistogram(const std::string &_filename, const std::string &_histogramTitle, const std::vector<Hist> &_histograms, const double _binSize, const double _lowerBound = -1, const double _upperBound = -1);
	static void writeOuputData(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const std::vector<BandPtr> &_bands, const std::vector<Hist> &_angleHistograms, const ExecutionParams &_params);
	static void writePlotSSE(const std::string &_filename, const std::string &_plotTitle, const std::vector<double> &_sse);
	static void writeClusteredCloud(const std::string &_filename, const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const cv::Mat &_labels);
	static void writeDistanceMatrix(const std::string &_filename, const cv::Mat &_items, const cv::Mat &_centers, const cv::Mat &_labels, const MetricPtr &_metric);

private:
	Writer();
	~Writer();

	static void generateHistogramScript(const std::string &_outputFolder, const std::string &_histogramTitle, const int _bandsNumber, const double _binSize, const double _lowerLimit, const double _upperLimit);

	static bool comparePairs(const std::pair<int, int> &_item1, const std::pair<int, int> &_item2)
	{
		return _item1.second < _item2.second;
	}
};