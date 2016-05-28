/**
 * Author: rodrigo
 * 2016
 */
#pragma once

#include <vector>
#include <string>
#include <stdint.h>
#include <opencv2/core/core.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "ExecutionParams.hpp"

// Colors defined to be used for points
typedef enum PointColor
{
	COLOR_FIREBRICK = 0xB22222,
	COLOR_RED = 0xFF0000,
	COLOR_SALMON = 0xFA8072,
	COLOR_ORANGE = 0xFFA500,
	COLOR_GOLDEN_ROD = 0xDAA520,
	COLOR_OLIVE = 0x808000,
	COLOR_KHAKI = 0xF0E68C,
	COLOR_YELLOW = 0xFFFF00,
	COLOR_DARK_GREEN = 0x006400,
	COLOR_GREEN = 0x008000,
	COLOR_LIME = 0x00FF00,
	COLOR_LIGHT_GREEN = 0x90EE90,
	COLOR_DARK_SEA_GREEN = 0x8FBC8F,
	COLOR_MEDIUM_AQUA = 0x66CDAA,
	COLOR_TEAL = 0x008080,
	COLOR_CYAN = 0x00FFFF,
	COLOR_TURQUOISE = 0x40E0D0,
	COLOR_DODGER_BLUE = 0x1E90FF,
	COLOR_SKY_BLUE = 0x87CEEB,
	COLOR_NAVY = 0x000080,
	COLOR_BLUE = 0x0000FF,
	COLOR_BLUE_VIOLET = 0x8A2BE2,
	COLOR_SLATE_BLUE = 0x6A5ACD,
	COLOR_DARK_MAGENTA = 0x8B008B,
	COLOR_PLUM = 0xDDA0DD,
	COLOR_MAGENTA = 0xFF00FF,
	COLOR_DEEP_PINK = 0xFF1493,
	COLOR_HOT_PINK = 0xFF69B4,
	COLOR_BEIGE = 0xF5F5DC,
	COLOR_BROWN = 0xA0522D,
	COLOR_ROSY_BROWN = 0xBC8F8F,
	COLOR_SLATE_GRAY = 0x708090,
	COLOR_BLACK = 0x000000,
	COLOR_GRAY = 0x808080,
	COLOR_SILVER = 0xC0C0C0,
	COLOR_WHITE = 0xFFFFFF,
} PointColor;

// Utils class definition
class Utils
{
public:
	// Returns the current application's working directory
	static std::string getWorkingDirectory();

	// Returns a string with the hex representation of the hash calculated for the current params instance
	static std::string getCalculationConfigHash(const std::string _inputCloudFile, const double _normalEstimationRadius, const DescriptorParams &_descriptorParams, const CloudSmoothingParams &_smoothingParams);

	// Returns the MD5 checksum for the named file
	static std::string getFileChecksum(const std::string _filename);

	// Returns a randomly generated integer between the given ranges
	static int getRandomNumber(const int _min, const int _max);

	// Returns an array with random numbers
	static std::vector<int> getRandomArray(const unsigned int _size, const int _min, const int _max);

	// Returns a string hex representation of the given number
	static std::string num2Hex(const size_t _number);

	// Returns a float representation fo the given RGB color
	static float getColor(const uint8_t _r, const uint8_t _g, const uint8_t _b);

	// Returns a color from the built-in color pallete (12 colors available)
	static uint32_t colorPalette12(const int _index);

	// Returns a color from the built-in color pallete (35 colors available)
	static uint32_t colorPalette35(const int _index);

	// Generates a pair of arbitrary points in the given plane, both defining a couple of perpendicular vectors when the difference from the plane's origin is used
	static std::pair<Eigen::Vector3f, Eigen::Vector3f> generatePerpendicularPointsInPlane(const Eigen::Hyperplane<float, 3> &_plane, const Eigen::Vector3f &_point);

	// Calculates the Sum of Squared Errors indicator for the given vectors over the given centers, using the given labeling
	static inline double getSSE(const cv::Mat &_vectors, const cv::Mat &_centers, const cv::Mat &_labels)
	{
		double sse = 0;
		for (int i = 0; i < _vectors.rows; i++)
		{
			float norm = cv::norm(_vectors.row(i), _centers.row(_labels.at<int>(i)));
			sse += (norm * norm);
		}

		return sse;
	}

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
			if (direction < 0)
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
