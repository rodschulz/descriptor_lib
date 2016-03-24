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

std::vector<int> Utils::getRandomArray(const unsigned int _size, const int _min, const int _max)
{
	generator.seed(std::rand());
	boost::random::uniform_int_distribution<> dist(_min, _max);

	std::vector<int> numbers;
	numbers.reserve(_size);

	std::map<int, bool> used;
	while (numbers.size() < _size)
	{
		int number = dist(generator);
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
	static uint32_t palette[12] =
	{ 0xa6cee3, 0x1f78b4, 0xb2df8a, 0x33a02c, 0xfb9a99, 0xe31a1c, 0xfdbf6f, 0xff7f00, 0xcab2d6, 0x6a3d9a, 0xffff99, 0xb15928 };

	return palette[_index % 12];
}

uint32_t Utils::colorPalette35(const int _index)
{
	static uint32_t palette[35] =
	{ COLOR_FIREBRICK, COLOR_GOLDEN_ROD, COLOR_DARK_GREEN, COLOR_MEDIUM_AQUA, COLOR_DODGER_BLUE, COLOR_BLUE_VIOLET, COLOR_PLUM, COLOR_BEIGE, COLOR_SLATE_GRAY,
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
