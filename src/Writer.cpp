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

void Writer::writeHistogram(const string &_filename, const string &_histogramTitle, const vector<Hist> &_histograms, const double _binSize, const double _lowerBound, const double _upperBound)
{
	if (!_histograms.empty())
	{
		bool axesCreated = false;
		vector<string> rows;
		ostringstream stream;
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
					stream.str(string());
					stream << (step * j + boundary);
					rows[j] = stream.str();
				}
				axesCreated = true;
			}

			for (int j = 0; j < binsNumber; j++)
			{
				stream.str(string());
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
		ofstream output;
		output.open(PLOT_DATA_NAME, fstream::out);
		for (size_t i = 0; i < rows.size(); i++)
			output << rows[i] << "\n";
		output.close();

		// Execute script to generate plot
		string cmd = "gnuplot ";
		cmd += PLOT_SCRIPT_NAME;
		if (system(cmd.c_str()) != 0)
			cout << "WARNING, bad return for command: " << cmd << "\n";

		// Remove script and data
		/*cmd = "rm -rf ";
		cmd += PLOT_SCRIPT_NAME;
		if (system(cmd.c_str()))
			cout << "WARNING, bad return for command: " << cmd << "\n";

		cmd = "rm -rf ";
		cmd += PLOT_DATA_NAME;
		if (system(cmd.c_str()))
			cout << "WARNING, bad return for command: " << cmd << "\n";*/
	}
}

void Writer::generateScript(const string &_filename, const string &_histogramTitle, const int _bandsNumber, const double _binSize, const double _lowerLimit, const double _upperLimit)
{
	ofstream output;
	output.open(PLOT_SCRIPT_NAME, fstream::out);

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
