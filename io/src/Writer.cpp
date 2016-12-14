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
#include <cstdlib>
#include <pcl/pcl_macros.h>
#include <pcl/io/pcd_io.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "CloudFactory.hpp"
#include "Utils.hpp"
#include "Config.hpp"


#define MATRIX_DIMENSIONS			"dims"
#define SCRIPT_HISTOGRAM_NAME       OUTPUT_DIR "histogramPlot.script"
#define SCRIPT_SSE_NAME             OUTPUT_DIR "ssePlot.script"
#define HISTOGRAM_DATA_FILE         OUTPUT_DIR "histogram.dat"
#define SSE_DATA_FILE               OUTPUT_DIR "sse.dat"


void Writer::writeHistogram(const std::string &filename_,
							const std::string &histogramTitle_,
							const std::vector<Hist> &histograms_,
							const double binSize_,
							const double lowerBound_,
							const double upperBound_)
{
	if (!histograms_.empty())
	{
		bool axesCreated = false;
		std::vector<std::string> rows;
		std::ostringstream stream;
		Dimension dimension = ANGLE;

		// Generate data to plot
		for (size_t i = 0; i < histograms_.size(); i++)
		{
			Bins bins;
			if (lowerBound_ == -1 || upperBound_ == -1)
				histograms_[i].getBins(binSize_, bins);
			else
				histograms_[i].getBins(binSize_, lowerBound_, upperBound_, bins);

			int binsNumber = bins.bins.size();

			// Create axes if not already created
			if (!axesCreated)
			{
				rows.resize(binsNumber);
				dimension = bins.dimension;

				double step = dimension == ANGLE ? RAD2DEG(bins.step) : bins.step;
				double boundary = dimension == ANGLE ? RAD2DEG(lowerBound_) : lowerBound_;
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
		double step = dimension == ANGLE ? RAD2DEG(binSize_) : binSize_;
		double lower = dimension == ANGLE ? RAD2DEG(lowerBound_) : lowerBound_;
		double upper = dimension == ANGLE ? RAD2DEG(upperBound_) : upperBound_;
		generateHistogramScript(filename_, histogramTitle_, histograms_.size(), step, lower, upper);

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

void Writer::generateHistogramScript(const std::string &filename_,
									 const std::string &histogramTitle_,
									 const int bandsNumber_,
									 const double binSize_,
									 const double lowerLimit_,
									 const double upperLimit_)
{
	std::ofstream output;
	output.open(SCRIPT_HISTOGRAM_NAME, std::fstream::out);

	output << "set title '" << histogramTitle_ << "'\n";
	output << "set ylabel 'Percentage'\n";
	output << "set xlabel 'Degrees'\n\n";

	output << "set xrange [" << lowerLimit_ << ":" << upperLimit_ << "]\n";
	output << "set yrange [0:1]\n";
	output << "set grid ytics xtics\n\n";

	output << "set xtics " << binSize_ << "\n";
	output << "set xtics font 'Verdana,9'\n";
	output << "set xtics rotate by -50\n";
	output << "set xtics (";

	int binNumber = ceil((upperLimit_ - lowerLimit_) / binSize_);
	double offset = binNumber % 2 > 0 ? 0 : -0.5 * binSize_;

	for (double pos = lowerLimit_ + offset; pos < upperLimit_; pos += binSize_)
		output << "'[" << round(pos) << ", " << round(pos + binSize_) << ")' " << pos - offset << (pos + binSize_ <= upperLimit_ ? ", " : "");
	output << ")\n\n";

	output << "set style data linespoints\n\n";

	output << "set term png  size 1024,768\n";
	output << "set output '" << OUTPUT_DIR << filename_ << ".png'\n\n";

	output << "plot \\\n";
	for (int i = 0; i < bandsNumber_; i++)
		output << "'" << HISTOGRAM_DATA_FILE
			   << "' using 1:" << i + 2 << " title 'Band "
			   << i
			   << "' with linespoints lw 2 lc rgb '#" << Utils::num2Hex(Utils::palette12(i + 1)) << "' pt 2"
			   << (i == bandsNumber_ - 1 ? "\n" : ", \\\n") ;

	output.close();
}

void Writer::writeOuputData(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_,
							const std::vector<BandPtr> &bands_,
							const std::vector<Hist> &angleHistograms_,
							const DescriptorParamsPtr &params_,
							const int targetPoint_)
{
	DCHParams *params = dynamic_cast<DCHParams *>(params_.get()); // FIX THIS !! => do something when the cast fails because is another descriptor


	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr coloredCloud = CloudFactory::createColorCloud(cloud_, Utils::palette12(0));
	pcl::io::savePCDFileASCII(OUTPUT_DIR "cloud.pcd", *coloredCloud);

	(*coloredCloud)[targetPoint_].rgba = Utils::getColor(255, 0, 0);
	pcl::io::savePCDFileASCII(OUTPUT_DIR "pointPosition.pcd", *coloredCloud);

	std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> planes = Extractor::generatePlaneClouds(bands_, params);

	pcl::PointCloud<pcl::PointNormal>::Ptr patch = Extractor::getNeighbors(cloud_, cloud_->at(targetPoint_), params->searchRadius);
	pcl::io::savePCDFileASCII(OUTPUT_DIR "patch.pcd", *patch);

	std::ofstream sequences;
	sequences.open(OUTPUT_DIR "sequences", std::fstream::out);

	for (size_t i = 0; i < bands_.size(); i++)
	{
		if (!bands_[i]->data->empty())
		{
			char name[100];
			sprintf(name, OUTPUT_DIR "band%d.pcd", (int) i);
			pcl::io::savePCDFileASCII(name, *CloudFactory::createColorCloud(bands_[i]->data, Utils::palette12(i + 1)));

			sequences << "band " << i << ": " << bands_[i]->sequenceString << "\n";

			sprintf(name, OUTPUT_DIR "planeBand%d.pcd", (int) i);
			pcl::io::savePCDFileASCII(name, *planes[i]);
		}
	}

	sequences.close();

	// Write histogram data
	double limit = M_PI;
	Writer::writeHistogram("angles", "Angle Distribution", angleHistograms_, DEG2RAD(20), -limit, limit);
}

void Writer::writePlotSSE(const std::string &filename_,
						  const std::string &plotTitle_,
						  const std::vector<double> &sse_)
{
	if (sse_.empty())
	{
		std::cout << "WARNING: array SSE empty, can't generate SSE plot" << std::endl;
		return;
	}

	// Generate data file
	std::ofstream dataFile;
	dataFile.open(SSE_DATA_FILE, std::fstream::out);
	for (size_t i = 0; i < sse_.size(); i++)
		dataFile << i << "\t" << sse_[i] << "\n";
	dataFile.close();

	// Generate plot script
	std::ofstream plotScript;
	plotScript.open(SCRIPT_SSE_NAME, std::fstream::out);

	plotScript << "set title '" << plotTitle_ << "'\n";
	plotScript << "set ylabel 'SSE'\n";
	plotScript << "set xlabel 'Iterations'\n\n";

	plotScript << "set grid ytics xtics\n";
	plotScript << "set style data linespoints\n\n";

	plotScript << "set term png\n";
	plotScript << "set output '" << OUTPUT_DIR << filename_ << ".png'\n\n";

	plotScript << "plot '" << SSE_DATA_FILE << "' using 1:2 title 'SSE'\n";

	plotScript.close();

	// Execute script to generate plot
	std::string cmd = "gnuplot ";
	cmd += SCRIPT_SSE_NAME;
	if (system(cmd.c_str()) != 0)
		std::cout << "WARNING, bad return for command: " << cmd << "\n";
}

void Writer::writeClusteredCloud(const std::string &filename_,
								 const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_,
								 const cv::Mat &labels_)
{
	// Color the data according to the clusters
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr colored = CloudFactory::createColorCloud(cloud_, Utils::palette35(0));
	for (int i = 0; i < labels_.rows; i++)
	{
		int index = 0;
		switch (labels_.type())
		{
		case CV_16U:
			index = (int) labels_.at<unsigned short>(i);
			break;

		case CV_16S:
			index = (int) labels_.at<short>(i);
			break;

		case CV_32S:
			index = labels_.at<int>(i);
			break;

		case CV_32F:
			index = (int) labels_.at<float>(i);
			break;

		case CV_64F:
			index  = (int) labels_.at<double>(i);
			break;

		default:
			std::cout << "WARNING: wrong label type" << std::endl;
		}

		(*colored)[i].rgba = Utils::palette35(index);
	}

	pcl::io::savePCDFileASCII(filename_, *colored);
}

void Writer::writeDistanceMatrix(const std::string &outputFolder_,
								 const cv::Mat &items_,
								 const cv::Mat &centers_,
								 const cv::Mat &labels_,
								 const MetricPtr &metric_)
{
	std::vector<std::pair<int, int> > data; // fist=index - second=cluster
	for (int i = 0; i < labels_.rows; i++)
	{
		data.push_back(std::make_pair(i, labels_.at<int>(i)));
	}
	std::sort(data.begin(), data.end(), comparePairs);

	// Generate a matrix of distances between points
	float maxDistanceBetween = 0;
	cv::Mat distBetweenPoints = cv::Mat::zeros(items_.rows, items_.rows, CV_32FC1);
	for (size_t i = 0; i < data.size(); i++)
	{
		int index1 = data[i].first;
		//for (size_t j = 0; j < data.size(); j++)
		for (size_t j = 0; j <= i; j++)
		{
			int index2 = data[j].first;
			float distance = (float) metric_->distance(items_.row(index1), items_.row(index2));
			distBetweenPoints.at<float>(i, j) = distance;
			distBetweenPoints.at<float>(j, i) = distance;

			maxDistanceBetween = distance > maxDistanceBetween ? distance : maxDistanceBetween;
		}
	}

	// Generate a matrix of distances to the centers
	int pixels = 40;
	float maxDistanceToCenter = 0;
	cv::Mat distToCenters = cv::Mat::zeros(items_.rows, centers_.rows * pixels, CV_32FC1);
	for (size_t i = 0; i < data.size(); i++)
	{
		int index = data[i].first;
		for (int j = 0; j < centers_.rows; j++)
		{
			float distance = (float) metric_->distance(items_.row(index), centers_.row(j));
			distToCenters.row(i).colRange(j * pixels, (j + 1) * pixels) = distance;
			maxDistanceToCenter = distance > maxDistanceToCenter ? distance : maxDistanceToCenter;
		}
	}

	// Write images
	cv::Mat imageDistBetweenPoints;
	distBetweenPoints.convertTo(imageDistBetweenPoints, CV_8UC1, 255 / maxDistanceBetween, 0);
	cv::imwrite(outputFolder_ + "distanceBetweenPoints.png", imageDistBetweenPoints);

	cv::Mat imageDistToCenter;
	distToCenters.convertTo(imageDistToCenter, CV_8UC1, 255 / maxDistanceToCenter, 0);
	cv::imwrite(outputFolder_ + "distanceToCenter.png", imageDistToCenter);
}

void Writer::writeDescriptorsCache(const cv::Mat &descriptors_,
								   const std::string &cacheLocation_,
								   const std::string &cloudInputFilename_,
								   const double normalEstimationRadius_,
								   const DescriptorParamsPtr &descriptorParams_,
								   const CloudSmoothingParams &smoothingParams_)
{
	if (!boost::filesystem::exists(cacheLocation_))
		if (system(("mkdir " + cacheLocation_).c_str()) != 0)
			std::cout << "WARNING: can't create cache folder" << std::endl;

	std::string destination = cacheLocation_ + Utils::getCalculationConfigHash(cloudInputFilename_, normalEstimationRadius_, descriptorParams_, smoothingParams_);

	std::vector<std::string> metadata;
	metadata.push_back("normalEstimationRadius:" + boost::lexical_cast<std::string>(normalEstimationRadius_));
	metadata.push_back(descriptorParams_->toString());
	metadata.push_back(smoothingParams_.toString());

	writeMatrix(destination, descriptors_, metadata);
}

void Writer::writeClustersCenters(const std::string &filename_,
								  const cv::Mat &centers_,
								  const DescriptorParamsPtr &descriptorParams_,
								  const ClusteringParams &clusteringParams_,
								  const CloudSmoothingParams &smoothingParams_)
{
	std::vector<std::string> metadata;
	metadata.push_back(descriptorParams_->toString());
	metadata.push_back(clusteringParams_.toString());
	metadata.push_back(smoothingParams_.toString());

	writeMatrix(filename_, centers_, metadata);
}

void Writer::writeBoW(const std::string &filename_,
					  const cv::Mat &centers_,
					  const ClusteringParams &clusteringParams_,
					  const int nbands_,
					  const int nbins_,
					  const bool bidirectional_)
{
	std::string str = "bandNumber: " + boost::lexical_cast<std::string>(nbands_) + " binNumber: " + boost::lexical_cast<std::string>(nbins_) + " bidirectional: " + (bidirectional_ ? "true" : "false");

	std::vector<std::string> metadata;
	metadata.push_back(clusteringParams_.toString());
	metadata.push_back(str);

	Writer::writeMatrix(filename_, centers_, metadata);
}

void Writer::saveCloudMatrix(const std::string &filename_,
							 const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_)
{
	cv::Mat items = cv::Mat::zeros(cloud_->size(), 3, CV_32FC1);
	for (size_t i = 0; i < cloud_->size(); i++)
	{
		items.at<float>(i, 0) = cloud_->at(i).x;
		items.at<float>(i, 1) = cloud_->at(i).y;
		items.at<float>(i, 2) = cloud_->at(i).z;
	}

	Writer::writeMatrix(filename_, items);
}

void Writer::writeMatrix(const std::string &filename_,
						 const cv::Mat &matrix_,
						 const std::vector<std::string> &metadata_)
{
	std::ofstream outputFile;
	outputFile.open(filename_.c_str(), std::fstream::out);

	// Write metadata header
	outputFile << "metadata_lines " << metadata_.size() << "\n";

	// Write metadata
	for (size_t i = 0; i < metadata_.size(); i++)
		outputFile << metadata_[i] << "\n";

	// Write the matrix actual data
	outputFile << MATRIX_DIMENSIONS << " " << matrix_.rows << " " << matrix_.cols << "\n";
	for (int i = 0; i < matrix_.rows; i++)
	{
		for (int j = 0; j < matrix_.cols; j++)
			outputFile << std::setprecision(15) << matrix_.at<float>(i, j) << " ";
		outputFile << "\n";
	}

	outputFile.close();
}
