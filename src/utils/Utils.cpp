/**
 * Author: rodrigo
 * 2016
 */
#include "Utils.hpp"
#include <ctime>
#include <sstream>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int_distribution.hpp>

boost::random::mt19937 randomGenerator;

int Utils::getRandomNumber(const int _min, const int _max)
{
	randomGenerator.seed(std::time(0));
	boost::random::uniform_int_distribution<> dist(_min, _max);
	return dist(randomGenerator);
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
