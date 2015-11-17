/**
 * Author: rodrigo
 * 2015
 */
#include "Loader.h"
#include <iostream>
#include <fstream>

Loader::Loader()
{
}

Loader::~Loader()
{
}

bool Loader::loadMatrix(cv::Mat &_matrix, const std::string &_filename)
{
	bool loadOk = true;
	size_t row = 0;

	try
	{
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

				loadOk = (int) tokens.size() == _matrix.cols;
				if (!loadOk)
					break;

				for (size_t col = 0; col < tokens.size(); col++)
					_matrix.at<float>(row, col) = (float) atof(tokens[col].c_str());
				row++;
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
