/**
 * Author: rodrigo
 * 2016
 */
#include "Utils.hpp"
#include <ctime>
#include <sstream>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int_distribution.hpp>

boost::random::mt19937 generator;

int Utils::getRandomNumber(const int _min, const int _max)
{
	generator.seed(std::time(0));
	boost::random::uniform_int_distribution<> dist(_min, _max);
	return dist(generator);
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

uint32_t Utils::getColor(const int _index)
{
	static uint32_t palette[12] =
	{ 0xa6cee3, 0x1f78b4, 0xb2df8a, 0x33a02c, 0xfb9a99, 0xe31a1c, 0xfdbf6f, 0xff7f00, 0xcab2d6, 0x6a3d9a, 0xffff99, 0xb15928 };

	return palette[_index % 12];
}

double Utils::getSSE(const cv::Mat &_vectors, const cv::Mat &_centers, const cv::Mat &_labels)
{
	double sse = 0;
	for (int i = 0; i < _vectors.rows; i++)
	{
		float norm = cv::norm(_vectors.row(i), _centers.row(_labels.at<int>(i)));
		sse += (norm * norm);
	}

	return sse;
}

std::pair<Eigen::Vector3f, Eigen::Vector3f> Utils::generatePlaneAxes(const Eigen::Hyperplane<float, 3> &_plane, const Eigen::Vector3f &_point)
{
	Eigen::Vector3f normal = _plane.normal();

	// Generates a pair of perpendicular vectors using an arbitrary direction for the director vector
	Eigen::Vector3f v1 = _point + Eigen::Vector3f(10, 10, 10);
	v1 = _plane.projection(v1).normalized();
	Eigen::Vector3f v2 = normal.cross(v1).normalized();

	return std::pair<Eigen::Vector3f, Eigen::Vector3f>(v1, v2);
}
