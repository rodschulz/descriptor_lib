/**
 * Author: rodrigo
 * 2016
 */
#pragma once

#include <vector>
#include <string>
#include <stdint.h>
#include <opencv2/core/core.hpp>

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

	// Calculates the Sum of Squared Errors indicator for the given vectors over the given centers, using the given labeling
	static double getSSE(const cv::Mat &_vectors, const cv::Mat &_centers, const cv::Mat &_labels);

	// Returns +1 if the sign of the given value is positive, -1 if it's negative, and 0 when it's 0
	template<typename T> static inline int sign(T val)
	{
		return (T(0) < val) - (val < T(0));
	}

	// Calculates the angle between the two given vectors
	template<class T> static inline double angle(const T &_vector1, const T &_vector2)
	{
		return atan2(_vector1.cross(_vector2).norm(), _vector1.dot(_vector2));
	}

	// Returns the signed angle between the two given vectors
	template<class T> static inline double signedAngle(const T &_vector1, const T &_vector2, const T &_normal)
	{
		double direction = _normal.dot(_vector1.cross(_vector2));

		// Check if the cross product is not zero
		if (fabs(direction) > 1E-7)
		{
			if (_normal.dot(_vector1.cross(_vector2)) < 0)
				return -atan2(_vector1.cross(_vector2).norm(), _vector1.dot(_vector2));
			else
				return atan2(_vector1.cross(_vector2).norm(), _vector1.dot(_vector2));
		}
		else
		{
			if (_vector1.dot(_vector2) >= 0)
				return 0;
			else
				return M_PI;
		}
	}

private:
	Utils();
	~Utils();
};
