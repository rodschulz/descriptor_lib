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
#include <plog/Log.h>
#include "CloudFactory.hpp"
#include "PointFactory.hpp"
#include "Utils.hpp"
#include "Config.hpp"
#include "DCH.hpp"


#define MATRIX_DIMENSIONS			"dims"
#define SCRIPT_HISTOGRAM_NAME       OUTPUT_DIR "histogramPlot.script"
#define SCRIPT_SSE_NAME             OUTPUT_DIR "ssePlot.script"
#define HISTOGRAM_DATA_FILE         OUTPUT_DIR "histogram.dat"
#define SSE_DATA_FILE               OUTPUT_DIR "sse.dat"


void Writer::writeHistogram(const std::string &filename_,
							const std::string &histogramTitle_,
							const std::vector<Histogram> &histograms_,
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


		// Precision for the plot data
		stream << std::fixed << std::setprecision(4);


		// Generate data to plot
		for (size_t i = 0; i < histograms_.size(); i++)
		{
			Bins b = (lowerBound_ == -1 || upperBound_ == -1)
					 ? histograms_[i].getBins(binSize_)
					 : histograms_[i].getBins(binSize_, lowerBound_, upperBound_);
			int binsNumber = b.bins.size();

			// Create axes if not already created
			if (!axesCreated)
			{
				rows.resize(binsNumber);
				dimension = b.dimension;

				double step = dimension == ANGLE ? RAD2DEG(b.step) : b.step;
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
				stream << "\t" << b.bins[j];
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
			LOGW << "Bad return for command: " << cmd;
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

	// int binNumber = ceil((upperLimit_ - lowerLimit_) / binSize_);
	// double offset = binNumber % 2 > 0 ? 0 : -0.5 * binSize_;
	double offset = 0;

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

std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr>
Writer::generatePlanes(const std::vector<BandPtr> &bands_,
					   const DCHParams *params_)
{
	std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> planes;
	planes.reserve(bands_.size());

	float delta = params_->searchRadius;
	float begin = params_->bidirectional ? -delta : 0;
	float step = (delta - begin) / 10;

	for (size_t i = 0; i < bands_.size(); i++)
	{
		Eigen::Vector3f point = bands_[i]->origin.getVector3fMap();
		Eigen::Vector3f normal = bands_[i]->plane.normal();
		planes.push_back(pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>()));

		for (float x = begin; x <= delta; x += step)
		{
			for (float y = begin; y <= delta; y += step)
			{
				for (float z = begin; z <= delta; z += step)
				{
					Eigen::Vector3f p = bands_[i]->plane.projection(point + Eigen::Vector3f(x, y, z));
					planes.back()->push_back((pcl::PointNormal) PointFactory::createPointNormal(p.x(), p.y(), p.z(), normal[0], normal[1], normal[2]));
				}
			}
		}
	}

	return planes;
}

void Writer::writeOuputData(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_,
							const std::vector<BandPtr> &bands_,
							const DescriptorParamsPtr &params_,
							const int targetPoint_)
{
	DCHParams *params = dynamic_cast<DCHParams *>(params_.get()); // FIX THIS !! => do something when the cast fails because is another descriptor


	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr colorCloud = CloudFactory::createColorCloud(cloud_, Utils::palette12(0));
	pcl::io::savePCDFileASCII(OUTPUT_DIR "cloud.pcd", *colorCloud);


	(*colorCloud)[targetPoint_].rgba = Utils::getColor(255, 0, 0);
	pcl::io::savePCDFileASCII(OUTPUT_DIR "pointPosition.pcd", *colorCloud);


	pcl::PointCloud<pcl::PointNormal>::Ptr patch = Extractor::getNeighbors(cloud_, cloud_->at(targetPoint_), params->searchRadius);
	pcl::io::savePCDFileASCII(OUTPUT_DIR "patch.pcd", *patch);


	std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> planes = generatePlanes(bands_, params);
	for (size_t i = 0; i < bands_.size(); i++)
	{
		if (!bands_[i]->points->empty())
		{
			char name[100];
			sprintf(name, OUTPUT_DIR "band%d.pcd", (int) i);
			pcl::io::savePCDFileASCII(name, *CloudFactory::createColorCloud(bands_[i]->points, Utils::palette12(i + 1)));

			sprintf(name, OUTPUT_DIR "planeBand%d.pcd", (int) i);
			pcl::io::savePCDFileASCII(name, *planes[i]);
		}
	}


	// Write histogram data
	std::vector<Histogram> angleHistograms = DCH::generateAngleHistograms(bands_, params->useProjection);
	// getBins(DEG2RAD(20), -M_PI / 2, M_PI / 2)
	Writer::writeHistogram("angles", "Angle Distribution", angleHistograms, DEG2RAD(20), -M_PI / 2, M_PI / 2);


	// Write the descriptor to a file
	std::ofstream output;
	output.open(OUTPUT_DIR "descriptor.dat", std::fstream::out);
	output << std::fixed << std::setprecision(4);
	for (size_t i = 0; i < bands_.size(); i++)
	{
		for (size_t j = 0; j < bands_.at(i)->descriptor.size(); j++)
			output << bands_.at(i)->descriptor[j] << "\t";
		output << "\n";
	}
	output.close();
}

void Writer::writePlotSSE(const std::string &filename_,
						  const std::string &plotTitle_,
						  const std::vector<double> &sse_)
{
	if (sse_.empty())
	{
		LOGW << "Array SSE empty, can't generate SSE plot";
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
		LOGW << "Bad return for command: " << cmd;
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
			LOGW << "Wrong label type";
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
			LOGW << "Can't create cache folder";

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

void Writer::writeCodebook(const std::string &filename_,
						   const cv::Mat &centers_,
						   const ClusteringParams &clusteringParams_,
						   const Params::DescriptorType &type_,
						   const float searchRadius_,
						   const int nbands_,
						   const int nbins_,
						   const bool bidirectional_)
{
	std::string str = "type:" + Params::descType[type_];
	switch (type_)
	{
	default:
		std::runtime_error("Wrong descriptor type to generate codebook");

	case Params::DESCRIPTOR_DCH:
		str += " searchRadius:" + boost::lexical_cast<std::string>(searchRadius_)
			   + " bandNumber:" + boost::lexical_cast<std::string>(nbands_)
			   + " binNumber:" + boost::lexical_cast<std::string>(nbins_)
			   + " bidirectional:" + (bidirectional_ ? "true" : "false");
		break;

	case Params::DESCRIPTOR_SHOT:
	case Params::DESCRIPTOR_USC:
	case Params::DESCRIPTOR_PFH:
	case Params::DESCRIPTOR_ROPS:
		str += " searchRadius:" + boost::lexical_cast<std::string>(searchRadius_);
		break;
	}

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
