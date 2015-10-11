/**
 * Author: rodrigo
 * 2015
 */
#include "Writer.h"
#include <iostream>
#include <fstream>
#include <math.h>
#include <sstream>
#include <stdlib.h>
#include <pcl/pcl_macros.h>
#include <pcl/io/pcd_io.h>
#include "../factories/CloudFactory.h"

#define SCRIPT_NAME		"plot.script"
#define DATA_NAME		"histogram.dat"
#define PLOT_SCRIPT_NAME	OUTPUT_FOLDER SCRIPT_NAME
#define PLOT_DATA_NAME		OUTPUT_FOLDER DATA_NAME

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

			// Create axes if not already done
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
		generateScript(_filename, _histogramTitle, _histograms.size(), step, lower, upper);

		// Generate data file
		std::ofstream output;
		output.open(PLOT_DATA_NAME, std::fstream::out);
		for (size_t i = 0; i < rows.size(); i++)
			output << rows[i] << "\n";
		output.close();

		// Execute script to generate plot
		std::string cmd = "gnuplot ";
		cmd += PLOT_SCRIPT_NAME;
		if (system(cmd.c_str()) != 0)
			std::cout << "WARNING, bad return for command: " << cmd << "\n";
	}
}

void Writer::generateScript(const std::string &_filename, const std::string &_histogramTitle, const int _bandsNumber, const double _binSize, const double _lowerLimit, const double _upperLimit)
{
	std::ofstream output;
	output.open(PLOT_SCRIPT_NAME, std::fstream::out);

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
		output << "'" << PLOT_DATA_NAME << "' using 1:" << i + 2 << " title 'Band " << i << "', \\\n";

	output.close();
}

void Writer::writeOuput(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const std::vector<BandPtr> &_bands, const std::vector<Hist> &_angleHistograms, const ExecutionParams &_params)
{
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr coloredCloud = CloudFactory::createColorCloud(_cloud, Helper::getColor(0));
	pcl::io::savePCDFileASCII("./output/cloud.pcd", *coloredCloud);

	(*coloredCloud)[_params.targetPoint].rgb = Helper::getColor(255, 0, 0);
	pcl::io::savePCDFileASCII("./output/pointPosition.pcd", *coloredCloud);

	std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> planes = Extractor::getBandPlanes(_bands, _params);

	pcl::PointCloud<pcl::PointNormal>::Ptr patch = Extractor::getNeighbors(_cloud, _cloud->at(_params.targetPoint), _params.patchSize);
	pcl::io::savePCDFileASCII("./output/patch.pcd", *patch);

	std::ofstream sequences;
	sequences.open("./output/sequences", std::fstream::out);

	for (size_t i = 0; i < _bands.size(); i++)
	{
		if (!_bands[i]->data->empty())
		{
			char name[100];
			sprintf(name, "./output/band%d.pcd", (int) i);
			pcl::io::savePCDFileASCII(name, *CloudFactory::createColorCloud(_bands[i]->data, Helper::getColor(i + 1)));

			sequences << "band " << i << ": " << _bands[i]->sequenceString << "\n";

			sprintf(name, "./output/planeBand%d.pcd", (int) i);
			pcl::io::savePCDFileASCII(name, *planes[i]);
		}
	}

	sequences.close();

	// Write histogram data
	double limit = M_PI;
	Writer::writeHistogram("angles", "Angle Distribution", _angleHistograms, DEG2RAD(20), -limit, limit);
}
