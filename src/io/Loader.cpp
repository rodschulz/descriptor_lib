/**
 * Author: rodrigo
 * 2015
 */
#include "Loader.hpp"

#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include "../utils/CloudUtils.hpp"

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

bool Loader::loadDescriptorsCache(cv::Mat &_descriptors, const ExecutionParams &_params)
{
	std::string filename = _params.cacheLocation + _params.getHash();
	return loadMatrix(_descriptors, filename);
}

bool Loader::loadClusterCenters(const std::string &_filename, cv::Mat &_centers)
{
	return loadMatrix(_centers, _filename);
}

bool Loader::loadCloud(pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const ExecutionParams &_params)
{
	bool loadOk = true;

	// Load cartesian data from disk
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ(new pcl::PointCloud<pcl::PointXYZ>());
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(_params.inputLocation, *cloudXYZ) != 0)
	{
		std::cout << "ERROR: Can't read file from disk (" << _params.inputLocation << ")\n";
		loadOk = false;
	}

	if (loadOk)
	{
		// Remove NANs
		CloudUtils::removeNANs(cloudXYZ);

		// Apply smoothing
		switch (_params.smoothingType)
		{
			case SMOOTHING_GAUSSIAN:
				std::cout << "Applying gaussian smoothing\n";
				cloudXYZ = CloudUtils::gaussianSmoothing(cloudXYZ, _params.gaussianSigma, _params.gaussianRadius);
				break;

			case SMOOTHING_MLS:
				std::cout << "Applying MLS smoothing\n";
				cloudXYZ = CloudUtils::MLSSmoothing(cloudXYZ, _params.mlsRadius);
				break;

			default:
				break;
		}

		// Estimate normals
		pcl::PointCloud<pcl::Normal>::Ptr normals = CloudUtils::estimateNormals(cloudXYZ, _params.normalEstimationRadius);

		// Deliver the cloud
		_cloud->clear();
		pcl::concatenateFields(*cloudXYZ, *normals, *_cloud);
	}

	return loadOk;

}
