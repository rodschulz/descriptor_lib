/**
 * Author: rodrigo
 * 2015
 */
#include "Loader.hpp"
#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include "CloudUtils.hpp"
#include "Utils.hpp"

bool Loader::loadMatrix(cv::Mat &_matrix, const std::string &_filename)
{
	bool loadOk = true;
	size_t row = 0;

	try
	{
		bool firstLine = true;
		std::string line;
		std::ifstream cacheFile;
		cacheFile.open(_filename.c_str(), std::fstream::in);
		if (cacheFile.is_open())
		{
			while (getline(cacheFile, line))
			{
				if (line.empty())
					continue;

				std::vector<std::string> tokens;
				std::istringstream iss(line);
				std::copy(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(), std::back_inserter(tokens));

				if (firstLine)
				{
					firstLine = false;
					int rows = atoi(tokens[1].c_str());
					int cols = atoi(tokens[2].c_str());
					_matrix = cv::Mat::zeros(rows, cols, CV_32FC1);
				}
				else
				{
					loadOk = (int) tokens.size() == _matrix.cols;
					if (!loadOk)
						break;

					for (size_t col = 0; col < tokens.size(); col++)
						_matrix.at<float>(row, col) = (float) atof(tokens[col].c_str());
					row++;
				}
			}
			cacheFile.close();
		}
		else
			loadOk = false;
	}
	catch (std::exception &_ex)
	{
		std::cout << "ERROR: " << _ex.what() << std::endl;
		loadOk = false;
	}
	return loadOk;
}

bool Loader::loadDescriptors(const std::string &_cacheLocation, const std::string &_inputFile, const double _normalEstimationRadius, const DescriptorParams &_descritorParams, const CloudSmoothingParams &_smoothingParams, cv::Mat &_descriptors)
{
	std::string filename = _cacheLocation + Utils::getCalculationConfigHash(_inputFile, _normalEstimationRadius, _descritorParams, _smoothingParams);
	return loadMatrix(_descriptors, filename);
}

bool Loader::loadCenters(const std::string &_filename, cv::Mat &_centers)
{
	return loadMatrix(_centers, _filename);
}

bool Loader::loadCloud(const std::string &_filename, const double _normalEstimationRadius, const CloudSmoothingParams &_params, pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud)
{
	// Load cartesian data from disk
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ(new pcl::PointCloud<pcl::PointXYZ>());
	bool loadOk = pcl::io::loadPCDFile<pcl::PointXYZ>(_filename, *cloudXYZ) == 0;

	if (loadOk)
	{
		// Remove NANs
		CloudUtils::removeNANs(cloudXYZ);

		// Apply smoothing
		if (_params.useSmoothing)
			cloudXYZ = CloudUtils::gaussianSmoothing(cloudXYZ, _params.sigma, _params.radius);

		// Estimate normals
		pcl::PointCloud<pcl::Normal>::Ptr normals = CloudUtils::estimateNormals(cloudXYZ, _normalEstimationRadius);

		// Deliver the cloud
		_cloud->clear();
		pcl::concatenateFields(*cloudXYZ, *normals, *_cloud);
	}

	return loadOk;
}
