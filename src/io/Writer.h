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
	static void writeOuput(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const std::vector<BandPtr> &_bands, const std::vector<Hist> &_angleHistograms, const ExecutionParams &_params);

private:
	Writer();
	~Writer();

	static void generateScript(const std::string &_filename, const std::string &_histogramTitle, const int _bandsNumber, const double _binSize, const double _lowerLimit, const double _upperLimit);
};
