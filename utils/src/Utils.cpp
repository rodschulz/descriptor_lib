/**
 * Author: rodrigo
 * 2016
 */
#include "Utils.hpp"
#include <ctime>
#include <sstream>
#include <boost/version.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/preprocessor/stringize.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/functional/hash.hpp>
#include <boost/lexical_cast.hpp>
#include <openssl/md5.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <stdio.h>

// Extract the current boost's minor version
#define BOOST_MINOR_VERSION (BOOST_VERSION %100)

// Include headers and define generator accordingly
#if BOOST_MINOR_VERSION <= 46
#include <boost/random/uniform_int.hpp>
#include <boost/random/variate_generator.hpp>
boost::mt19937 generator;
#else
#include <boost/random/uniform_int_distribution.hpp>
boost::random::mt19937 generator;
#endif

std::string Utils::getWorkingDirectory()
{
	char workingDir[1024];
	if (getcwd(workingDir, sizeof(workingDir)) == NULL)
		std::cout << "WARNING: can't get working directory location" << std::endl;

	int len = strlen(workingDir);
	workingDir[len] = '/';
	workingDir[len + 1] = '\0';

	return workingDir;
}

std::string Utils::getCalculationConfigHash(const std::string _inputFile, const double _normalEstimationRadius, const DescriptorParams &_descriptorParams, const CloudSmoothingParams &_smoothingParams)
{
	std::string MD5 = getFileChecksum(_inputFile);

	// TODO improve this using the file's hash instead of just the file's name
	std::string filename = _inputFile;
	if (_inputFile.substr(0, 2).compare("./") == 0)
		filename = _inputFile.substr(2);

	std::string str = "";
	str += "input=" + filename;
	str += "-normalEstimationRadius=" + boost::lexical_cast<std::string>(_normalEstimationRadius);
	str += "-patchSize=" + boost::lexical_cast<std::string>(_descriptorParams.patchSize);
	str += "-bandNumber=" + boost::lexical_cast<std::string>(_descriptorParams.bandNumber);
	str += "-bandWidth=" + boost::lexical_cast<std::string>(_descriptorParams.bandWidth);
	str += "-bidirectional=" + boost::lexical_cast<std::string>(_descriptorParams.bidirectional);
	str += "-useProjection=" + boost::lexical_cast<std::string>(_descriptorParams.useProjection);
	str += "-sequenceBin=" + boost::lexical_cast<std::string>(_descriptorParams.sequenceBin);
	str += "-sequenceStat=" + boost::lexical_cast<std::string>(_descriptorParams.sequenceStat);
	if (_smoothingParams.useSmoothing)
	{
		str += "-useSmoothing=" + boost::lexical_cast<std::string>(_smoothingParams.useSmoothing);
		str += "-smoothingSigma=" + boost::lexical_cast<std::string>(_smoothingParams.sigma);
		str += "-smoothingRadius=" + boost::lexical_cast<std::string>(_smoothingParams.radius);
	}

	boost::hash<std::string> strHash;
	return Utils::num2Hex(strHash(str));
}

std::string Utils::getFileChecksum(const std::string _filename)
{
	int fileDescriptor = open(_filename.c_str(), O_RDONLY);
	if (fileDescriptor < 0)
		throw std::runtime_error("ERROR: Unable to open file for MD5 checksum");

	struct stat statbuf;
	if (fstat(fileDescriptor, &statbuf) < 0)
		throw std::runtime_error("ERROR: Unable to get file properties for MD5 checksum");

	unsigned long fileSize = statbuf.st_size;
	char *fileBuffer = (char *) mmap(0, fileSize, PROT_READ, MAP_SHARED, fileDescriptor, 0);

	unsigned char digest[MD5_DIGEST_LENGTH];
	MD5((unsigned char*) fileBuffer, fileSize, digest);
	munmap(fileBuffer, fileSize);

	char stringMD5[MD5_DIGEST_LENGTH * 2 + 1];
	for (int i = 0; i < MD5_DIGEST_LENGTH; i++)
		sprintf(&stringMD5[i * 2], "%02x", digest[i]);

	close(fileDescriptor);

	return stringMD5;
}

int Utils::getRandomNumber(const int _min, const int _max)
{
	generator.seed(std::time(0));

#if BOOST_MINOR_VERSION <= 46
	boost::uniform_int<> range(_min, _max);
	boost::variate_generator<boost::mt19937&, boost::uniform_int<> > dist(generator, range);
	return dist();
#else
	boost::random::uniform_int_distribution<> dist(_min, _max);
	return dist(generator);
#endif
}

std::vector<int> Utils::getRandomArray(const unsigned int _size, const int _min, const int _max)
{
	generator.seed(std::rand());

#if BOOST_MINOR_VERSION <= 46
	boost::uniform_int<> range(_min, _max);
	boost::variate_generator<boost::mt19937&, boost::uniform_int<> > dist(generator, range);
#else
	boost::random::uniform_int_distribution<> dist(_min, _max);
#endif

	std::vector<int> numbers;
	numbers.reserve(_size);

	std::map<int, bool> used;
	while (numbers.size() < _size)
	{
#if BOOST_MINOR_VERSION <= 46
		int number = dist();
#else
		int number = dist(generator);
#endif

		if (used.find(number) == used.end())
			numbers.push_back(number);
	}

	return numbers;
}

std::string Utils::num2Hex(const size_t _number)
{
	std::stringstream stream;
	stream << std::hex << _number;
	return stream.str();
}

float Utils::getColor(const uint8_t _r, const uint8_t _g, const uint8_t _b)
{
	uint32_t color = ((uint32_t) _r << 16 | (uint32_t) _g << 8 | (uint32_t) _b);
	float finalColor = *reinterpret_cast<float*>(&color);
	return finalColor;
}

uint32_t Utils::colorPalette12(const int _index)
{
	static uint32_t palette[12] = { 0xa6cee3, 0x1f78b4, 0xb2df8a, 0x33a02c, 0xfb9a99, 0xe31a1c, 0xfdbf6f, 0xff7f00, 0xcab2d6, 0x6a3d9a, 0xffff99, 0xb15928 };

	return palette[_index % 12];
}

uint32_t Utils::colorPalette35(const int _index)
{
	static uint32_t palette[35] = { COLOR_FIREBRICK, COLOR_GOLDEN_ROD, COLOR_DARK_GREEN, COLOR_MEDIUM_AQUA, COLOR_DODGER_BLUE, COLOR_BLUE_VIOLET, COLOR_PLUM, COLOR_BEIGE, COLOR_SLATE_GRAY,
	///
	COLOR_SALMON, COLOR_OLIVE, COLOR_GREEN, COLOR_TEAL, COLOR_SKY_BLUE, COLOR_SLATE_BLUE, COLOR_MAGENTA, COLOR_BROWN, COLOR_GRAY,
	///
	COLOR_RED, COLOR_KHAKI, COLOR_LIME, COLOR_CYAN, COLOR_NAVY, COLOR_DARK_MAGENTA, COLOR_DEEP_PINK, COLOR_SILVER,
	///
	COLOR_ORANGE, COLOR_YELLOW, COLOR_LIGHT_GREEN, COLOR_DARK_SEA_GREEN, COLOR_TURQUOISE, COLOR_BLUE, COLOR_HOT_PINK, COLOR_ROSY_BROWN, COLOR_WHITE };

	return palette[_index % 35];
}

std::pair<Eigen::Vector3f, Eigen::Vector3f> Utils::generatePerpendicularPointsInPlane(const Eigen::Hyperplane<float, 3> &_plane, const Eigen::Vector3f &_point)
{
	Eigen::Vector3f normal = _plane.normal();

	// Take an arbitrary direction from the plane's origin (OUTSIDE the plane)
	Eigen::Vector3f u = _point + Eigen::Vector3f(1E15, 1E15, 1E15);

	// Project that arbitrary point into the plane to get the first axis inside the plane
	Eigen::Vector3f v1 = _plane.projection(u).normalized();

	// Generate the seconde unitary vector
	Eigen::Vector3f v2 = normal.cross(v1).normalized();

	// Return the axes
	return std::pair<Eigen::Vector3f, Eigen::Vector3f>(v1, v2);
}
