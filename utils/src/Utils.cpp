/**
 * Author: rodrigo
 * 2016
 */
#include "Utils.hpp"
#include <ctime>
#include <sstream>
#include <boost/version.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/lagged_fibonacci.hpp>
#include <boost/preprocessor/stringize.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/functional/hash.hpp>
#include <boost/lexical_cast.hpp>
#include <openssl/md5.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <stdio.h>
#include <plog/Log.h>
#include <yaml-cpp/yaml.h>
#include "Config.hpp"


// Extract the current boost's minor version
#define BOOST_MINOR_VERSION (BOOST_VERSION %100)

// Include headers and define generator accordingly
#if BOOST_MINOR_VERSION <= 46
#include <boost/random/uniform_int.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>
boost::mt19937 generator;
#else
#include <boost/random/uniform_int_distribution.hpp>
boost::random::mt19937 generator;
#endif


template<typename T1, typename Generator, typename NumberType>
std::vector<T1> generateRandomSet(const unsigned int size_,
								  const T1 min_,
								  const T1 max_,
								  const bool allowRepetition_)
{
	generator.seed(std::rand());

#if BOOST_MINOR_VERSION <= 46
	NumberType range(min_, max_);
	boost::variate_generator<Generator&, NumberType> dist(generator, range);
#else
	Generator dist(min_, max_);
#endif

	std::vector<T1> numbers;
	numbers.reserve(size_);

	std::map<T1, bool> used;
	while (numbers.size() < size_)
	{
#if BOOST_MINOR_VERSION <= 46
		T1 number = dist();
#else
		T1 number = dist(generator);
#endif

		if (allowRepetition_ || used.find(number) == used.end())
		{
			used[number] = true;
			numbers.push_back(number);
		}
	}

	return numbers;
}

plog::Severity Utils::getLogLevel(const std::string &filename_)
{
	YAML::Node logging = YAML::LoadFile(filename_);
	return plog::severityFromString(logging["level"].as<std::string>().c_str());
}

void Utils::cleanDirectories(const std::string &workingDir_)
{
	// Create output and debug directories if don't exist
	if (system("mkdir -p " OUTPUT_DIR) != 0)
		throw std::runtime_error("Can't create directory: " + workingDir_ + OUTPUT_DIR);
	if (system("mkdir -p " DEBUG_DIR) != 0)
		throw std::runtime_error("Can't create directory: " + workingDir_ + DEBUG_DIR);

	// Clean output and debug directories
	if (system("rm -rf " OUTPUT_DIR "*") != 0)
		LOGW << "Can't clean directory: " + workingDir_ + OUTPUT_DIR;
	if (system("rm -rf " DEBUG_DIR "*") != 0)
		LOGW << "Can't clean directory: " + workingDir_ + DEBUG_DIR;
}

std::string Utils::getWorkingDirectory()
{
	char workingDir[1024];
	if (getcwd(workingDir, sizeof(workingDir)) == NULL)
		LOGW << "Can't get working directory location";

	int len = strlen(workingDir);
	workingDir[len] = '/';
	workingDir[len + 1] = '\0';

	return workingDir;
}

std::string Utils::getCalculationConfigHash(const std::string inputCloudFile_,
		const double normalEstimationRadius_,
		const DescriptorParamsPtr &descriptorParams_,
		const CloudSmoothingParams &smoothingParams_)
{
	std::string str = "";
	str += "input=" + getFileChecksum(inputCloudFile_);
	str += "-normalEstimationRadius=" + boost::lexical_cast<std::string>(normalEstimationRadius_);
	str += "-" + descriptorParams_->toString();
	if (smoothingParams_.useSmoothing)
		str += "-" + smoothingParams_.toString();

	boost::hash<std::string> strHash;
	return Utils::num2Hex(strHash(str));
}

std::string Utils::getFileChecksum(const std::string filename_)
{
	int fileDescriptor = open(filename_.c_str(), O_RDONLY);
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

int Utils::getRandomNumber(const int min_,
						   const int max_)
{
	generator.seed(std::time(0));

#if BOOST_MINOR_VERSION <= 46
	boost::uniform_int<> range(min_, max_);
	boost::variate_generator<boost::mt19937&, boost::uniform_int<> > dist(generator, range);
	return dist();
#else
	boost::random::uniform_int_distribution<> dist(min_, max_);
	return dist(generator);
#endif
}

std::vector<float> Utils::getRandomRealArray(const unsigned int size_,
		const float min_,
		const float max_,
		const bool allowRepetition_)
{
#if BOOST_MINOR_VERSION <= 46
	return generateRandomSet<float, boost::mt19937, boost::uniform_real<> >(size_, min_, max_, allowRepetition_);
#else
	return generateRandomSet<float, boost::random::uniform_real_distribution<>, void>(size_, min_, max_, allowRepetition_);
#endif
}

std::vector<int> Utils::getRandomIntArray(const unsigned int size_,
		const int min_,
		const int max_,
		const bool allowRepetition_)
{
#if BOOST_MINOR_VERSION <= 46
	return generateRandomSet<int, boost::mt19937, boost::uniform_int<> >(size_, min_, max_, allowRepetition_);
#else
	return generateRandomSet<int, boost::random::uniform_int_distribution<>, void>(size_, min_, max_, allowRepetition_);
#endif
}

std::string Utils::num2Hex(const size_t number_)
{
	std::stringstream stream;
	stream << std::hex << number_;
	return stream.str();
}

std::pair<Eigen::Vector3f, Eigen::Vector3f>
Utils::generateAxes(const Eigen::Hyperplane<float, 3> &plane_,
					const Eigen::Vector3f &point_)
{
	Eigen::Vector3f normal = plane_.normal();

	// Take an arbitrary direction from the plane's origin (OUTSIDE the plane)
	Eigen::Vector3f u = point_ + Eigen::Vector3f(1E15, 1E15, 1E15);

	// Project that arbitrary point into the plane to get the first axis inside the plane
	Eigen::Vector3f v1 = plane_.projection(u).normalized();

	// Generate the second unitary vector
	Eigen::Vector3f v2 = normal.cross(v1).normalized();

	// Return the axes
	return std::pair<Eigen::Vector3f, Eigen::Vector3f>(v1, v2);
}
