/**
 * Author: rodrigo
 * 2015
 */
#include "Loader.hpp"
#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <boost/lexical_cast.hpp>
#include <plog/Log.h>
#include "CloudUtils.hpp"
#include "Utils.hpp"


enum ReadingState
{
	READING_METADATA,
	READING_DIMENSIONS,
	READING_DATA
};


bool Loader::loadMatrix(const std::string &filename_,
						cv::Mat &matrix_,
						std::map<std::string, std::string> *metadata_)
{
	bool loadOk = true;
	size_t row = 0;

	try
	{
		ReadingState state = READING_METADATA;
		int metadataLines = -1;
		int metadataLinesRead = 0;
		std::string line;
		std::ifstream cacheFile;
		cacheFile.open(filename_.c_str(), std::fstream::in);
		if (cacheFile.is_open())
		{
			while (getline(cacheFile, line))
			{
				if (line.empty())
					continue;

				std::vector<std::string> tokens;
				std::istringstream iss(line);
				std::copy(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(), std::back_inserter(tokens));

				switch (state)
				{
				case READING_METADATA:
					if (metadataLines < 0)
						metadataLines = atoi(tokens[1].c_str());
					else
					{
						if (metadata_ != NULL)
						{
							// Parse metadata
							for (size_t i = 0; i < tokens.size(); i++)
							{
								std::vector<std::string> parts;
								boost::split(parts, tokens[i], boost::is_any_of(":"), boost::token_compress_on);
								metadata_->operator [](parts[0]) = parts[1];
							}
						}
						metadataLinesRead++;
					}

					if (metadataLinesRead >= metadataLines)
						state = READING_DIMENSIONS;
					break;

				case READING_DIMENSIONS:
					matrix_ = cv::Mat::zeros(atoi(tokens[1].c_str()), atoi(tokens[2].c_str()), CV_32FC1);
					state = READING_DATA;
					break;

				case READING_DATA:
					loadOk = (int) tokens.size() == matrix_.cols;
					if (!loadOk)
						break;

					for (size_t col = 0; col < tokens.size(); col++)
					{
						// Attempt to covert the strings into numbers
						float value = 0;
						try
						{
							value = boost::lexical_cast<float>(tokens[col]);
						}
						catch (boost::bad_lexical_cast ex_)
						{
							LOGW << "NaN found at (r,c) = (" << row << ", " << col << "). Changing to zero.";
						}

						matrix_.at<float>(row, col) = value;
					}
					row++;

					break;
				}
			}
			cacheFile.close();
		}
		else
			loadOk = false;
	}
	catch (std::exception &_ex)
	{
		LOGE << "ERROR: " << _ex.what();
		loadOk = false;
	}
	return loadOk;
}

bool Loader::loadDescriptors(const std::string &cacheLocation_,
							 const std::string &cloudInputFilename_,
							 const double normalEstimationRadius_,
							 const DescriptorParamsPtr &descritorParams_,
							 const CloudSmoothingParams &smoothingParams_,
							 cv::Mat &descriptors_)
{
	std::string filename = cacheLocation_ + Utils::getCalculationConfigHash(cloudInputFilename_, normalEstimationRadius_, descritorParams_, smoothingParams_);
	return loadMatrix(filename, descriptors_);
}

bool Loader::loadCloud(const std::string &filename_,
					   const double normalEstimationRadius_,
					   const CloudSmoothingParams &params_,
					   pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_)
{
	// Load cartesian data from disk
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ(new pcl::PointCloud<pcl::PointXYZ>());
	bool loadOk = pcl::io::loadPCDFile<pcl::PointXYZ>(filename_, *cloudXYZ) == 0;

	if (loadOk)
	{
		// Remove NANs
		CloudUtils::removeNANs(cloudXYZ);

		// Apply smoothing
		if (params_.useSmoothing)
			cloudXYZ = CloudUtils::gaussianSmoothing(cloudXYZ, params_.sigma, params_.radius);

		// Estimate normals
		pcl::PointCloud<pcl::Normal>::Ptr normals = CloudUtils::estimateNormals(cloudXYZ, normalEstimationRadius_);

		// Deliver the cloud
		cloud_->clear();
		pcl::concatenateFields(*cloudXYZ, *normals, *cloud_);
	}

	return loadOk;
}

void Loader::traverseDirectory(const std::string &inputDirectory_,
							   std::vector<std::pair<cv::Mat, std::map<std::string, std::string> > > &data_,
							   std::pair<int, int> &dimensions_)
{
	boost::filesystem::path target(inputDirectory_);
	boost::filesystem::directory_iterator it(target), eod;
	BOOST_FOREACH(boost::filesystem::path const & filePath, std::make_pair(it, eod))
	{
		if (is_regular_file(filePath))
		{
			if (boost::iequals(filePath.extension().string(), ".dat"))
			{
				LOGI << "Loading: " << filePath.string();

				cv::Mat centers;
				std::map<std::string, std::string> metadata;
				if (loadMatrix(filePath.string(), centers, &metadata))
				{
					data_.push_back(std::make_pair(centers, metadata));
					dimensions_.first += centers.rows;
					dimensions_.second = std::max(dimensions_.second, centers.cols);
				}
				else
					LOGI << "...failed, skipping to next file";
			}
		}
		else
			traverseDirectory(filePath.string(), data_, dimensions_);
	}
}
