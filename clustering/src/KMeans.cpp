/**
 * Author: rodrigo
 * 2015
 */
#include "KMeans.hpp"
#include <limits>
#include <fstream>
#include <iostream>
#include <iomanip>
#include "Utils.hpp"
#include "Config.hpp"
#include "ClusteringUtils.hpp"

/********** DEBUG DATA GENERATION METHODS **********/
#define DEBUG_CENTER_TITLE		false
#define DEBUG_FOLDER			"./output/"
#define DEBUG_DATA_EXT			".dat"
#define DEBUG_PLOT_SCRIPT		"plot.script"
#define DEBUG_PLOT_DATA			"plotItems"
#define DEBUG_PLOT_CENTERS		"plotCenters" DEBUG_DATA_EXT
#define DEBUG_PLOT_CENTERS_IMG	"plotCentersImg" DEBUG_DATA_EXT

inline std::pair<std::pair<float, float>, std::pair<float, float> > DEBUG_getLimits(const cv::Mat &items_)
{
	float minx, miny, maxx, maxy;
	minx = miny = std::numeric_limits<float>::max();
	maxx = maxy = -std::numeric_limits<float>::max();
	for (int i = 0; i < items_.rows; i++)
	{
		if (minx > items_.at<float>(i, 0))
			minx = items_.at<float>(i, 0);
		if (maxx < items_.at<float>(i, 0))
			maxx = items_.at<float>(i, 0);

		if (miny > items_.at<float>(i, 1))
			miny = items_.at<float>(i, 1);
		if (maxy < items_.at<float>(i, 1))
			maxy = items_.at<float>(i, 1);
	}

	return std::pair<std::pair<float, float>, std::pair<float, float> >(std::pair<float, float>(minx, maxx), std::pair<float, float>(miny, maxy));
}

void DEBUG_generateImage(const std::string &title_, const cv::Mat &items_, const cv::Mat &centers_, const cv::Mat &labels_, const std::pair<std::pair<float, float>, std::pair<float, float> > &limits_, const int attempt_)
{
	static int img = 0;
	static int lastAttempt = 0;

	if (lastAttempt != attempt_)
		img = 0;

	// Write centers to file
	std::fstream centersFile(DEBUG_FOLDER DEBUG_PLOT_CENTERS, std::ofstream::out);
	std::fstream centersImgFile(DEBUG_FOLDER DEBUG_PLOT_CENTERS_IMG, std::ofstream::out);
	for (int i = 0; i < centers_.rows; i++)
	{
		centersFile << std::fixed << std::setprecision(6) << centers_.at<float>(i, 0) << " " << centers_.at<float>(i, 1) << "\t";
		centersImgFile << std::fixed << std::setprecision(6) << centers_.at<float>(i, 1) << " " << centers_.at<float>(i, 0) << "\t";
	}
	centersFile << std::endl;
	centersImgFile << std::endl;
	centersFile.close();
	centersImgFile.close();

	// Generate files to write data
	std::vector<int> nPerCenter(centers_.rows);
	std::vector<std::fstream *> files;
	files.reserve(centers_.rows);
	for (int i = 0; i < centers_.rows; i++)
		files.push_back(new std::fstream(((std::string) DEBUG_FOLDER DEBUG_PLOT_DATA + boost::lexical_cast<std::string>(i) + DEBUG_DATA_EXT).c_str(), std::ofstream::out));

	// Write data to files
	for (int i = 0; i < items_.rows; i++)
	{
		(*files[labels_.at<int>(i)]) << std::fixed << std::setprecision(6) << items_.at<float>(i, 0) << " " << items_.at<float>(i, 1) << "\n";
		nPerCenter[labels_.at<int>(i)]++;
	}

	// Close files
	for (std::vector<std::fstream *>::iterator it = files.begin(); it != files.end(); it++)
		(*it)->close();

	// Prepare limits to plot
	float displayFactor = 1.25;
	float minx = limits_.first.first;
	float maxx = limits_.first.second;
	float deltax = maxx - minx > 0 ? maxx - minx : 1;
	float middlex = (minx + maxx) / 2;
	minx = middlex - deltax * displayFactor;
	maxx = middlex + deltax * displayFactor;

	float miny = limits_.second.first;
	float maxy = limits_.second.second;
	float deltay = maxy - miny > 0 ? maxy - miny : 1;
	float middley = (miny + maxy) / 2;
	miny = middley - deltay * displayFactor;
	maxy = middley + deltay * displayFactor;

	// Generate script
	std::fstream scriptFile(DEBUG_FOLDER DEBUG_PLOT_SCRIPT, std::ofstream::out);
	scriptFile << "set title '" << title_ << " - img:" << img << " - att:" << attempt_ << "'\n";
	scriptFile << "set xlabel 'x'\n";
	scriptFile << "set ylabel 'y'\n\n";

	scriptFile << "set xrange [" << minx << ":" << maxx << "]\n";
	scriptFile << "set yrange [" << miny << ":" << maxy << "]\n";
	scriptFile << "set grid ytics xtics\n\n";

	scriptFile << "set key outside\n";
	scriptFile << "set terminal pngcairo dashed size 1280,720 enhanced font 'Verdana,9'\n";
	scriptFile << "set output '" DEBUG_FOLDER << std::setfill('0') << std::setw(2) << attempt_ << "_" << std::setw(4) << img << ".png'\n\n";

	scriptFile << "plot \\\n";
	scriptFile << "x title 'Identity' lt 2 lc rgb 'black', \\\n";

	for (int i = 0; i < centers_.rows; i++)
		if (nPerCenter[i] > 0)
			scriptFile << "'" DEBUG_FOLDER DEBUG_PLOT_DATA << i << DEBUG_DATA_EXT "' using 1:2 title 'cluster " << i << "' pt 7 ps 1 lc rgb '#" << Utils::num2Hex(Utils::palette35(i)) << "', \\\n";

	for (int i = 0; i < centers_.rows; i++)
		scriptFile << "'" DEBUG_FOLDER DEBUG_PLOT_CENTERS "' using " << (2 * i) + 1 << ":" << (2 * i) + 2 << (DEBUG_CENTER_TITLE ? " title 'center " + boost::lexical_cast<std::string>(i) + "'" : "notitle") << " pt 2 ps 2 lw 2 lc rgb '#" << Utils::num2Hex(Utils::palette35(i)) << "', \\\n";

	for (int i = 0; i < centers_.rows; i++)
		scriptFile << "'" DEBUG_FOLDER DEBUG_PLOT_CENTERS_IMG "' using " << (2 * i) + 1 << ":" << (2 * i) + 2 << (DEBUG_CENTER_TITLE ? " title 'center " + boost::lexical_cast<std::string>(i) + "'" : "notitle") << " pt 1 ps 2 lw 2 lc rgb '#" << Utils::num2Hex(Utils::palette35(i)) << "'" << (i == centers_.rows - 1 ? "" : ", \\") << "\n";

	scriptFile.close();

	std::string cmd = "gnuplot " DEBUG_FOLDER DEBUG_PLOT_SCRIPT;
	if (system(cmd.c_str()) != 0)
		std::cout << "WARNING, bad return for command: " << cmd << "\n";

	img++;
}

/********** DEBUG DATA GENERATION METHODS **********/

void KMeans::searchClusters(ClusteringResults &_results, const cv::Mat &_items, const MetricPtr &_metric, const int _ncluster, const int _attempts, const int _maxIterations, const double _stopThreshold)
{
	KMeans::run(_results, _items, _metric, _ncluster, _attempts, _maxIterations, _stopThreshold);
}

void KMeans::stochasticSearchClusters(ClusteringResults &_results, const cv::Mat &_items, const MetricPtr &_metric, const int _ncluster, const int _attempts, const int _maxIterations, const double _stopThreshold, const int _sampleSize)
{
	KMeans::run(_results, _items, _metric, _ncluster, _attempts, _maxIterations, _stopThreshold, _sampleSize);
}

void KMeans::run(ClusteringResults &results_, const cv::Mat &items_, const MetricPtr &metric_, const int ncluster_, const int attempts_, const int maxIterations_, const double stopThreshold_, const int sampleSize_)
{
	bool debug = Config::debugEnabled() && items_.cols == 2;
	std::pair<std::pair<float, float>, std::pair<float, float> > limits;

	if (debug)
		limits = DEBUG_getLimits(items_);

	results_.prepare(ncluster_, items_.rows, items_.cols);
	std::vector<int> itemsPerCenter;

	double minSSE = std::numeric_limits<double>::max();
	for (int i = 0; i < attempts_; i++)
	{
		std::cout << "\t** attempt " << i << std::endl;

		cv::Mat centers = cv::Mat::zeros(ncluster_, items_.cols, CV_32FC1);
		cv::Mat sample = sampleSize_ == -1 ? items_ : cv::Mat::zeros(sampleSize_, items_.cols, CV_32SC1);
		cv::Mat labels = cv::Mat::zeros(sample.rows, 1, CV_32SC1);
		std::vector<double> sseCurve;

		// Select some of the elements as the initial centers
		getSample(items_, centers);
		metric_->validateCenters(centers);

		//if (debug)
		//	ClusteringUtils::print<float>(centers);

		// Iterate until the desired max iterations
		std::vector<int> itemCount;
		for (int j = 0; j < maxIterations_; j++)
		{
			if (j % 50 == 0)
				std::cout << "\t\tit:" << j << std::endl;

			// Select sample points (if needed)
			if (sampleSize_ != -1)
				getSample(items_, sample);

			// Set labels
			for (int k = 0; k < sample.rows; k++)
				labels.at<int>(k) = findClosestCenter(sample.row(k), centers, metric_);

			if (debug)
			{
				//ClusteringUtils::print<int>(labels);
				DEBUG_generateImage("Initial labeling", sample, centers, labels, limits, i);
			}

			// Store SSE evolution
			sseCurve.push_back(KMeans::getSSE(items_, labels, centers, metric_));

			// Updated centers and check if to stop
			if (updateCenters(itemCount, centers, sample, labels, ncluster_, stopThreshold_, metric_))
			{
				std::cout << "\tthreshold reached --> [attempt: " << i << " - iteration: " << j << "]" << std::endl;
				break;
			}

			if (debug)
			{
				//ClusteringUtils::print<float>(centers);
				DEBUG_generateImage("Updated centers", sample, centers, labels, limits, i);
			}
		}

		double sse = KMeans::getSSE(items_, labels, centers, metric_);
		std::cout << "\tSSE: " << std::fixed << sse << std::endl;

		if (debug)
			DEBUG_generateImage("Final state", sample, centers, labels, limits, i);

		if (sse < minSSE)
		{
			// Update the results
			labels.copyTo(results_.labels);
			centers.copyTo(results_.centers);
			results_.errorEvolution = sseCurve;

			// Update the control variables
			itemsPerCenter = itemCount;
			minSSE = sse;
		}
	}

	// Print a short report of the results
	std::cout << "KMeans finished -- SSE: " << minSSE << "\n";
	for (size_t i = 0; i < itemsPerCenter.size(); i++)
		std::cout << "\tcluster " << i << ": " << itemsPerCenter[i] << " points\n";
}

bool KMeans::updateCenters(std::vector<int> &_itemCount, cv::Mat &_centers, const cv::Mat &_sample, const cv::Mat _labels, const int _ncluster, const double _stopThreshold, const MetricPtr &_metric)
{
	// Calculate updated positions for the centers
	cv::Mat newCenters = _metric->calculateCenters(_ncluster, _sample, _labels, _itemCount);

	// Check if the "displacement" threshold was reached
	bool stop = evaluateStopCondition(_centers, newCenters, _stopThreshold, _metric);

	// Update centers
	newCenters.copyTo(_centers);

	// Return the stop
	return stop;
}

bool KMeans::evaluateStopCondition(const cv::Mat &_oldCenters, const cv::Mat &_newCenters, const double _threshold, const MetricPtr &_metric)
{
	bool thresholdReached = true;
	for (int k = 0; k < _oldCenters.rows && thresholdReached; k++)
		thresholdReached = thresholdReached && (_metric->distance(_oldCenters.row(k), _newCenters.row(k)) < _threshold);

	return thresholdReached;
}

int KMeans::findClosestCenter(const cv::Mat &_vector, const cv::Mat &_centers, const MetricPtr &_metric)
{
	int closestCenter = -1;

	//std::cout << "vector\n";
	//ClusteringUtils::print<float>(_vector);

	// Find the closest centroid for the current descriptor
	double minDist = std::numeric_limits<double>::max();
	for (int i = 0; i < _centers.rows; i++)
	{
		double distance = _metric->distance(_vector, _centers.row(i));

		//std::cout << "center (d=" << distance << ")\n";
		//ClusteringUtils::print<float>(_centers.row(i));

		if (distance < minDist)
		{
			minDist = distance;
			closestCenter = i;
		}
	}

	return closestCenter;
}

void KMeans::getSample(const cv::Mat &_items, cv::Mat &_sample)
{
	std::vector<int> randomSet = Utils::getRandomIntArray(_sample.rows, 0, _items.rows - 1, false);
	for (int j = 0; j < _sample.rows; j++)
		_items.row(randomSet[j]).copyTo(_sample.row(j));
}
