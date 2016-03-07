/**
 * Author: rodrigo
 * 2016
 */
#pragma once

#include <vector>
#include <string>
#include <stdint.h>

class Utils
{
public:
	// Returns a randomly generated integer between the given ranges
	static int getRandomNumber(const int _min, const int _max);
	// Returns a string hex representation of the given number
	static std::string num2Hex(const size_t _number);
	// Returns a float representation fo the given RGB color
	static float getColor(const uint8_t _r, const uint8_t _g, const uint8_t _b);
	// Returns a color from the built-in color pallete (12 colors available)
	static uint32_t getColor(const int _index);

private:
	Utils() {};
	~Utils() {};
};
