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

#define RAD2DEG(x)		((x) * 57.2957795131)
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

void Writer::writeData(const string &_filename, const vector<double> &_curvatures, const vector<Hist> &_curvatureHistograms, const vector<Hist> &_angleHistograms)
{
	string name = OUTPUT_FOLDER + _filename;

	fstream output;
	output.open(name.c_str(), fstream::out);
	for (size_t i = 0; i < _curvatures.size(); i++)
		output << "Curvature band " << i << ": " << _curvatures[i] << "\n";

	output << "Curvature histograms\n";
	for (size_t i = 0; i < _curvatureHistograms.size(); i++)
	{
		Bins bins;
		_curvatureHistograms[i].getBins(8, bins);
		output << "BAND" << i << ": " << bins;
	}
	output << "Angle Histograms\n";
	for (size_t i = 0; i < _angleHistograms.size(); i++)
	{
		Bins bins;
		_angleHistograms[i].getBins(18, 0, M_PI, bins);
		output << "BAND" << i << ": " << bins;
	}
	output.close();
}

void Writer::writeHistogram(const string &_filename, const string &_histogramTitle, const vector<Hist> &_histograms, const int _binsNumber, const double _lowerBound, const double _upperBound)
{
	if (!_histograms.empty())
	{
		// Generate the plotting script
		generateScript(_filename, _histogramTitle, _histograms.size());

		ostringstream stream;
		vector<string> rows(_binsNumber, "");

		// Generate the data to plot
		Bins aux;
		(_lowerBound == -1 || _upperBound == -1) ? _histograms[0].getBins(_binsNumber, aux) : _histograms[0].getBins(_binsNumber, _lowerBound, _upperBound, aux);

		double step = aux.dimension == ANGLE ? RAD2DEG(aux.step) : aux.step;
		for (int j = 0; j < _binsNumber; j++)
		{
			stream.str(string());
			stream << step * j;
			rows[j] = stream.str();
		}

		for (size_t i = 0; i < _histograms.size(); i++)
		{
			Bins bins;
			(_lowerBound == -1 || _upperBound == -1) ? _histograms[i].getBins(_binsNumber, bins) : _histograms[i].getBins(_binsNumber, _lowerBound, _upperBound, bins);

			for (int j = 0; j < _binsNumber; j++)
			{
				stream.str(string());
				stream << "\t" << bins.bins[j];
				rows[j] += stream.str();
			}
		}

		// Generate data file
		ofstream output;
		output.open(PLOT_DATA_NAME, fstream::out);
		for (size_t i = 0; i < rows.size(); i++)
			output << rows[i] << "\n";
		output.close();

		// Execute script to generate plot
		string cmd = "gnuplot ";
		cmd += PLOT_SCRIPT_NAME;
		system(cmd.c_str());

		// Remove script and data
		cmd = "rm -rf ";
		cmd += PLOT_SCRIPT_NAME;
		system(cmd.c_str());
		cmd = "rm -rf ";
		cmd += PLOT_DATA_NAME;
		system(cmd.c_str());
	}
}

void Writer::generateScript(const string &_filename, const string &_histogramTitle, const int _bandsNumber)
{
	ofstream output;
	output.open(PLOT_SCRIPT_NAME, fstream::out);

	output << "set title '" << _histogramTitle << "'\n";
	output << "set ylabel 'Percentage'\n";
	output << "set xlabel 'Degrees'\n";

	output << "set auto x\n";
	output << "set yrange [0:1]\n";
	output << "set style data linespoints\n";

	output << "set term png\n";
	output << "set output '" << OUTPUT_FOLDER << _filename << ".png'\n";

	output << "plot \\\n";
	for (int i = 0; i < _bandsNumber; i++)
		output << "'" << PLOT_DATA_NAME << "' using 1:" << i + 2 << " title 'Band " << i << "', \\\n";

	output.close();
}
