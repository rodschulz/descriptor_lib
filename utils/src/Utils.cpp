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
	Generator dist(_min, _max);
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

std::string Utils::getCalculationConfigHash(const std::string _inputCloudFile,
		const double normalEstimationRadius_,
		const DescriptorParams &descriptorParams_,
		const CloudSmoothingParams &smoothingParams_)
{
	std::string str = "";
	str += "input=" + getFileChecksum(_inputCloudFile);
	str += "-normalEstimationRadius=" + boost::lexical_cast<std::string>(normalEstimationRadius_);
	str += "-" + descriptorParams_.toString();
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

int Utils::getRandomNumber(const int _min,
						   const int _max)
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

std::string Utils::num2Hex(const size_t _number)
{
	std::stringstream stream;
	stream << std::hex << _number;
	return stream.str();
}

std::pair<Eigen::Vector3f, Eigen::Vector3f> Utils::generatePerpendicularPointsInPlane(const Eigen::Hyperplane<float, 3> &plane_,
		const Eigen::Vector3f &point_)
{
	Eigen::Vector3f normal = plane_.normal();

	// Take an arbitrary direction from the plane's origin (OUTSIDE the plane)
	Eigen::Vector3f u = point_ + Eigen::Vector3f(1E15, 1E15, 1E15);

	// Project that arbitrary point into the plane to get the first axis inside the plane
	Eigen::Vector3f v1 = plane_.projection(u).normalized();

	// Generate the seconde unitary vector
	Eigen::Vector3f v2 = normal.cross(v1).normalized();

	// Return the axes
	return std::pair<Eigen::Vector3f, Eigen::Vector3f>(v1, v2);
}

SynCloudType Utils::getSynCloudType(const std::string &_type)
{
	if (boost::iequals(_type, "cube"))
		return CLOUD_CUBE;
	else if (boost::iequals(_type, "cylinder"))
		return CLOUD_CYLINDER;
	else if (boost::iequals(_type, "sphere"))
		return CLOUD_SPHERE;
	else if (boost::iequals(_type, "half_sphere"))
		return CLOUD_HALF_SPHERE;
	else if (boost::iequals(_type, "plane"))
		return CLOUD_PLANE;
	{
		std::cout << "WARNING: wrong synthetic cloud type, assuming SPHERE";
		return CLOUD_SPHERE;
	}
}

SequenceStat Utils::getStatType(const std::string &_type)
{
	if (boost::iequals(_type, "mean"))
		return STAT_MEAN;
	else if (boost::iequals(_type, "median"))
		return STAT_MEDIAN;
	{
		std::cout << "WARNING: wrong stat type, assuming MEAN";
		return STAT_MEAN;
	}
}

ClusteringImplementation Utils::getClusteringImplementation(const std::string &_type)
{
	if (boost::iequals(_type, "opencv"))
		return CLUSTERING_OPENCV;
	else if (boost::iequals(_type, "kmeans"))
		return CLUSTERING_KMEANS;
	else if (boost::iequals(_type, "stochastic"))
		return CLUSTERING_STOCHASTIC;
	else if (boost::iequals(_type, "kmedoids"))
		return CLUSTERING_KMEDOIDS;
	else
	{
		std::cout << "WARNING: wrong clustering implementation, assuming OPENCV";
		return CLUSTERING_OPENCV;
	}
}

MetricType Utils::getMetricType(const std::string &_type)
{
	if (boost::iequals(_type, "euclidean"))
		return METRIC_EUCLIDEAN;
	else if (boost::iequals(_type, "closest"))
		return METRIC_CLOSEST_PERMUTATION;
	else if (boost::iequals(_type, "closest_with_confidence"))
		return METRIC_CLOSEST_PERMUTATION_WITH_CONFIDENCE;
	{
		std::cout << "WARNING: wrong metric type, assuming EUCLIDEAN";
		return METRIC_EUCLIDEAN;
	}
}
