/**
 * Author: rodrigo
 * 2015
 */
#include "Writer.hpp"

#include <iostream>
#include <fstream>
#include <math.h>
#include <sstream>
#include <stdlib.h>
#include <pcl/pcl_macros.h>
#include <pcl/io/pcd_io.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "../factories/CloudFactory.hpp"
#include "../utils/Utils.hpp"

#define SCRIPT_HISTOGRAM_NAME	OUTPUT_FOLDER "histogramPlot.script"
#define SCRIPT_SSE_NAME		OUTPUT_FOLDER "ssePlot.script"
#define HISTOGRAM_DATA_FILE	OUTPUT_FOLDER "histogram.dat"
#define SSE_DATA_FILE		OUTPUT_FOLDER "sse.dat"

Writer::Writer()
{
}

Writer::~Writer()
{
}

void Writer::writeHistogram(const std::string &_filename, const std::string &_histogramTitle, const std::vector<Hist> &_histograms, const double _binSize, const double _lowerBound, const double _upperBound)
{
	if (!_histograms.empty())
	{
		bool axesCreated = false;
		std::vector<std::string> rows;
		std::ostringstream stream;
		Dimension dimension = ANGLE;

		// Generate data to plot
		for (size_t i = 0; i < _histograms.size(); i++)
		{
			Bins bins;
			if (_lowerBound == -1 || _upperBound == -1)
				_histograms[i].getBins(_binSize, bins);
			else
				_histograms[i].getBins(_binSize, _lowerBound, _upperBound, bins);

			int binsNumber = bins.bins.size();

			// Create axes if not already created
			if (!axesCreated)
			{
				rows.resize(binsNumber);
				dimension = bins.dimension;

				double step = dimension == ANGLE ? RAD2DEG(bins.step) : bins.step;
				double boundary = dimension == ANGLE ? RAD2DEG(_lowerBound) : _lowerBound;
				for (int j = 0; j < binsNumber; j++)
				{
					stream.str(std::string());
					stream << (step * j + boundary);
					rows[j] = stream.str();
				}
				axesCreated = true;
			}

			for (int j = 0; j < binsNumber; j++)
			{
				stream.str(std::string());
				stream << "\t" << bins.bins[j];
				rows[j] += stream.str();
			}
		}

		// Generate the plotting script
		double step = dimension == ANGLE ? RAD2DEG(_binSize) : _binSize;
		double lower = dimension == ANGLE ? RAD2DEG(_lowerBound) : _lowerBound;
		double upper = dimension == ANGLE ? RAD2DEG(_upperBound) : _upperBound;
		generateHistogramScript(_filename, _histogramTitle, _histograms.size(), step, lower, upper);

		// Generate data file
		std::ofstream output;
		output.open(HISTOGRAM_DATA_FILE, std::fstream::out);
		for (size_t i = 0; i < rows.size(); i++)
			output << rows[i] << "\n";
		output.close();

		// Execute script to generate plot
		std::string cmd = "gnuplot ";
		cmd += SCRIPT_HISTOGRAM_NAME;
		if (system(cmd.c_str()) != 0)
			std::cout << "WARNING, bad return for command: " << cmd << "\n";
	}
}

void Writer::generateHistogramScript(const std::string &_filename, const std::string &_histogramTitle, const int _bandsNumber, const double _binSize, const double _lowerLimit, const double _upperLimit)
{
	std::ofstream output;
	output.open(SCRIPT_HISTOGRAM_NAME, std::fstream::out);

	output << "set title '" << _histogramTitle << "'\n";
	output << "set ylabel 'Percentage'\n";
	output << "set xlabel 'Degrees'\n\n";

	output << "set xrange [" << _lowerLimit << ":" << _upperLimit << "]\n";
	output << "set yrange [0:1]\n";
	output << "set grid ytics xtics\n\n";

	output << "set xtics " << _binSize << "\n";
	output << "set xtics font 'Verdana,9'\n";
	output << "set xtics rotate by -50\n";
	output << "set xtics (";

	int binNumber = ceil((_upperLimit - _lowerLimit) / _binSize);
	double offset = binNumber % 2 > 0 ? 0 : -0.5 * _binSize;

	for (double pos = _lowerLimit + offset; pos < _upperLimit; pos += _binSize)
		output << "'[" << round(pos) << ", " << round(pos + _binSize) << ")' " << pos - offset << (pos + _binSize <= _upperLimit ? ", " : "");
	output << ")\n\n";

	output << "set style data linespoints\n\n";

	output << "set term png\n";
	output << "set output '" << OUTPUT_FOLDER << _filename << ".png'\n\n";

	output << "plot \\\n";
	for (int i = 0; i < _bandsNumber; i++)
		output << "'" << HISTOGRAM_DATA_FILE << "' using 1:" << i + 2 << " title 'Band " << i << "', \\\n";

	output.close();
}

void Writer::writeOuputData(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const std::vector<BandPtr> &_bands, const std::vector<Hist> &_angleHistograms, const ExecutionParams &_params)
{
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr coloredCloud = CloudFactory::createColorCloud(_cloud, Utils::colorPalette35(0));
	pcl::io::savePCDFileASCII(OUTPUT_FOLDER "cloud.pcd", *coloredCloud);

	(*coloredCloud)[_params.targetPoint].rgb = Utils::getColor(255, 0, 0);
	pcl::io::savePCDFileASCII(OUTPUT_FOLDER "pointPosition.pcd", *coloredCloud);

	std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> planes = Extractor::generatePlaneClouds(_bands, _params);

	pcl::PointCloud<pcl::PointNormal>::Ptr patch = Extractor::getNeighbors(_cloud, _cloud->at(_params.targetPoint), _params.patchSize);
	pcl::io::savePCDFileASCII(OUTPUT_FOLDER "patch.pcd", *patch);

	std::ofstream sequences;
	sequences.open(OUTPUT_FOLDER "sequences", std::fstream::out);

	for (size_t i = 0; i < _bands.size(); i++)
	{
		if (!_bands[i]->data->empty())
		{
			char name[100];
			sprintf(name, OUTPUT_FOLDER "band%d.pcd", (int) i);
			pcl::io::savePCDFileASCII(name, *CloudFactory::createColorCloud(_bands[i]->data, Utils::colorPalette35(i + 1)));

			sequences << "band " << i << ": " << _bands[i]->sequenceString << "\n";

			sprintf(name, OUTPUT_FOLDER "planeBand%d.pcd", (int) i);
			pcl::io::savePCDFileASCII(name, *planes[i]);
		}
	}

	sequences.close();

	// Write histogram data
	double limit = M_PI;
	Writer::writeHistogram("angles", "Angle Distribution", _angleHistograms, DEG2RAD(20), -limit, limit);
}

void Writer::writePlotSSE(const std::string &_filename, const std::string &_plotTitle, const std::vector<double> &_sse)
{
	if (_sse.empty())
	{
		std::cout << "WARNING: array SSE empty, can't generate SSE plot" << std::endl;
		return;
	}

	// Generate data file
	std::ofstream dataFile;
	dataFile.open(SSE_DATA_FILE, std::fstream::out);
	for (size_t i = 0; i < _sse.size(); i++)
		dataFile << i << "\t" << _sse[i] << "\n";
	dataFile.close();

	// Generate plot script
	std::ofstream plotScript;
	plotScript.open(SCRIPT_SSE_NAME, std::fstream::out);

	plotScript << "set title '" << _plotTitle << "'\n";
	plotScript << "set ylabel 'SSE'\n";
	plotScript << "set xlabel 'Iterations'\n\n";

	plotScript << "set grid ytics xtics\n";
	plotScript << "set style data linespoints\n\n";

	plotScript << "set term png\n";
	plotScript << "set output '" << OUTPUT_FOLDER << _filename << ".png'\n\n";

	plotScript << "plot '" << SSE_DATA_FILE << "' using 1:2 title 'SSE'\n";

	plotScript.close();

	// Execute script to generate plot
	std::string cmd = "gnuplot ";
	cmd += SCRIPT_SSE_NAME;
	if (system(cmd.c_str()) != 0)
		std::cout << "WARNING, bad return for command: " << cmd << "\n";
}

void Writer::writeClusteredCloud(const std::string &_filename, const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const cv::Mat &_labels)
{
	// Color the data according to the clusters
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr colored = CloudFactory::createColorCloud(_cloud, Utils::colorPalette35(0));
	for (int i = 0; i < _labels.rows; i++)
		(*colored)[i].rgb = Utils::colorPalette35(_labels.at<int>(i));

	pcl::io::savePCDFileASCII(_filename, *colored);
}

void Writer::writeDistanceMatrix(const std::string &_outputFolder, const cv::Mat &_items, const cv::Mat &_centers, const cv::Mat &_labels, const MetricPtr &_metric)
{
	std::vector<std::pair<int, int> > data; // fist=index - second=cluster
	for (int i = 0; i < _labels.rows; i++)
	{
		data.push_back(std::make_pair(i, _labels.at<int>(i)));
	}
	std::sort(data.begin(), data.end(), comparePairs);

	// Generate a matrix of distances between points
	float maxDistanceBetween = 0;
	cv::Mat distBetweenPoints = cv::Mat::zeros(_items.rows, _items.rows, CV_32FC1);
	for (size_t i = 0; i < data.size(); i++)
	{
		int index1 = data[i].first;
		//for (size_t j = 0; j < data.size(); j++)
		for (size_t j = 0; j <= i; j++)
		{
			int index2 = data[j].first;
			float distance = (float) _metric->distance(_items.row(index1), _items.row(index2));
			distBetweenPoints.at<float>(i, j) = distance;
			distBetweenPoints.at<float>(j, i) = distance;

			maxDistanceBetween = distance > maxDistanceBetween ? distance : maxDistanceBetween;
		}
	}

	// Generate a matrix of distances to the centers
	int pixels = 40;
	float maxDistanceToCenter = 0;
	cv::Mat distToCenters = cv::Mat::zeros(_items.rows, _centers.rows * pixels, CV_32FC1);
	for (size_t i = 0; i < data.size(); i++)
	{
		int index = data[i].first;
		for (int j = 0; j < _centers.rows; j++)
		{
			float distance = (float) _metric->distance(_items.row(index), _centers.row(j));
			distToCenters.row(i).colRange(j * pixels, (j + 1) * pixels) = distance;
			maxDistanceToCenter = distance > maxDistanceToCenter ? distance : maxDistanceToCenter;
		}
	}

	// Write images
	cv::Mat imageDistBetweenPoints;
	distBetweenPoints.convertTo(imageDistBetweenPoints, CV_8UC1, 255 / maxDistanceBetween, 0);
	cv::imwrite(_outputFolder + "distanceBetweenPoints.png", imageDistBetweenPoints);

	cv::Mat imageDistToCenter;
	distToCenters.convertTo(imageDistToCenter, CV_8UC1, 255 / maxDistanceToCenter, 0);
	cv::imwrite(_outputFolder + "distanceToCenter.png", imageDistToCenter);
}

void Writer::writeDescriptorsCache(const cv::Mat &_descriptors, const ExecutionParams &_params)
{
	if (!boost::filesystem::exists(_params.cacheLocation))
		if (system(("mkdir " + _params.cacheLocation).c_str()) != 0)
			std::cout << "WARNING: can't create clustering cache folder" << std::endl;

	std::string destination = _params.cacheLocation + _params.getHash();
	writeMatrix(destination, _descriptors);
}

void Writer::writeClustersCenters(const std::string &_outputFolder, const cv::Mat &_centers)
{
	// TODO add extra info when a cache is being saved, maybe parameters used for creation like num iterations, stop, patch size, etc

	std::string destination = _outputFolder + "centers";
	writeMatrix(destination, _centers);
}

void Writer::writeMatrix(const std::string &_filename, const cv::Mat &_matrix)
{
	std::ofstream outputFile;
	outputFile.open(_filename.c_str(), std::fstream::out);

	outputFile << MATRIX_DIMENSIONS << " " << _matrix.rows << " " << _matrix.cols << "\n";
	for (int i = 0; i < _matrix.rows; i++)
	{
		for (int j = 0; j < _matrix.cols; j++)
			outputFile << std::setprecision(15) << _matrix.at<float>(i, j) << " ";
		outputFile << "\n";
	}

	outputFile.close();
}
