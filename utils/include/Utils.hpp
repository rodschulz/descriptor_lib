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
	COLOR_FIREBRICK = 0x00B22222,
	COLOR_RED = 0x00FF0000,
	COLOR_SALMON = 0x00FA8072,
	COLOR_ORANGE = 0x00FFA500,
	COLOR_GOLDEN_ROD = 0x00DAA520,
	COLOR_OLIVE = 0x00808000,
	COLOR_KHAKI = 0x00F0E68C,
	COLOR_YELLOW = 0x00FFFF00,
	COLOR_DARK_GREEN = 0x00006400,
	COLOR_GREEN = 0x00008000,
	COLOR_LIME = 0x0000FF00,
	COLOR_LIGHT_GREEN = 0x0090EE90,
	COLOR_DARK_SEA_GREEN = 0x008FBC8F,
	COLOR_MEDIUM_AQUA = 0x0066CDAA,
	COLOR_TEAL = 0x00008080,
	COLOR_CYAN = 0x0000FFFF,
	COLOR_TURQUOISE = 0x0040E0D0,
	COLOR_DODGER_BLUE = 0x001E90FF,
	COLOR_SKY_BLUE = 0x0087CEEB,
	COLOR_NAVY = 0x00000080,
	COLOR_BLUE = 0x000000FF,
	COLOR_BLUE_VIOLET = 0x008A2BE2,
	COLOR_SLATE_BLUE = 0x006A5ACD,
	COLOR_DARK_MAGENTA = 0x008B008B,
	COLOR_PLUM = 0x00DDA0DD,
	COLOR_MAGENTA = 0x00FF00FF,
	COLOR_DEEP_PINK = 0x00FF1493,
	COLOR_HOT_PINK = 0x00FF69B4,
	COLOR_BEIGE = 0x00F5F5DC,
	COLOR_BROWN = 0x00A0522D,
	COLOR_ROSY_BROWN = 0x00BC8F8F,
	COLOR_SLATE_GRAY = 0x00708090,
	COLOR_BLACK = 0x00000000,
	COLOR_GRAY = 0x00808080,
	COLOR_SILVER = 0x00C0C0C0,
	COLOR_WHITE = 0x00FFFFFF,

	//Color brewer definitions
	COLOR_BREWER_QUALITATIVE_SKYBLUE = 0xA6CEE3,
	COLOR_BREWER_QUALITATIVE_BLUE = 0x1F78B4,
	COLOR_BREWER_QUALITATIVE_LIGHTGREEN = 0xB2DF8A,
	COLOR_BREWER_QUALITATIVE_GREEN = 0x33A02C,
	COLOR_BREWER_QUALITATIVE_PINK = 0xFB9A99,
	COLOR_BREWER_QUALITATIVE_RED = 0xE31A1C,
	COLOR_BREWER_QUALITATIVE_SALMON = 0xFDBF6F,
	COLOR_BREWER_QUALITATIVE_ORANGE = 0xFF7F00,
	COLOR_BREWER_QUALITATIVE_LAVANDA = 0xCAB2D6,
	COLOR_BREWER_QUALITATIVE_PURPLE = 0x6A3D9A,
	COLOR_BREWER_QUALITATIVE_LIGHTYELLOW = 0xFFFF99,
	COLOR_BREWER_QUALITATIVE_BROWN = 0xB15928,
} PointColor;


// Color palettes definitions
static uint32_t colorPalette12[12] = {
	COLOR_BREWER_QUALITATIVE_SKYBLUE,
	COLOR_BREWER_QUALITATIVE_BLUE,
	COLOR_BREWER_QUALITATIVE_LIGHTGREEN,
	COLOR_BREWER_QUALITATIVE_GREEN,
	COLOR_BREWER_QUALITATIVE_PINK,
	COLOR_BREWER_QUALITATIVE_RED,
	COLOR_BREWER_QUALITATIVE_SALMON,
	COLOR_BREWER_QUALITATIVE_ORANGE,
	COLOR_BREWER_QUALITATIVE_LAVANDA,
	COLOR_BREWER_QUALITATIVE_PURPLE,
	COLOR_BREWER_QUALITATIVE_LIGHTYELLOW,
	COLOR_BREWER_QUALITATIVE_BROWN
};

static uint32_t colorPalette35[35] = {
	COLOR_FIREBRICK,
	COLOR_GOLDEN_ROD,
	COLOR_DARK_GREEN,
	COLOR_MEDIUM_AQUA,
	COLOR_DODGER_BLUE,
	COLOR_BLUE_VIOLET,
	COLOR_PLUM,
	COLOR_BEIGE,
	COLOR_SLATE_GRAY,
	COLOR_SALMON,
	COLOR_OLIVE,
	COLOR_GREEN,
	COLOR_TEAL,
	COLOR_SKY_BLUE,
	COLOR_SLATE_BLUE,
	COLOR_MAGENTA,
	COLOR_BROWN,
	COLOR_GRAY,
	COLOR_RED,
	COLOR_KHAKI,
	COLOR_LIME,
	COLOR_CYAN,
	COLOR_NAVY,
	COLOR_DARK_MAGENTA,
	COLOR_DEEP_PINK,
	COLOR_SILVER,
	COLOR_ORANGE,
	COLOR_YELLOW,
	COLOR_LIGHT_GREEN,
	COLOR_DARK_SEA_GREEN,
	COLOR_TURQUOISE,
	COLOR_BLUE,
	COLOR_HOT_PINK,
	COLOR_ROSY_BROWN,
	COLOR_WHITE
};


// Utils class definition
class Utils
{
public:
	/**************************************************/
	static std::string getWorkingDirectory();

	/**************************************************/
	static std::string getCalculationConfigHash(const std::string _inputCloudFile,
			const double normalEstimationRadius_,
			const DescriptorParams &descriptorParams_,
			const CloudSmoothingParams &smoothingParams_);

	/**************************************************/
	static std::string getFileChecksum(const std::string filename_);

	/**************************************************/
	static int getRandomNumber(const int _min,
							   const int _max);

	/**************************************************/
	static std::vector<float> getRandomRealArray(const unsigned int size_,
			const float min_,
			const float max_,
			const bool allowRepetition_);

	/**************************************************/
	static std::vector<int> getRandomIntArray(const unsigned int _size,
			const int _min,
			const int _max,
			const bool allowRepetition_);

	/**************************************************/
	static std::string num2Hex(const size_t _number);

	/**************************************************/
	static inline uint32_t getColor(const uint8_t _r,
									const uint8_t _g,
									const uint8_t _b)
	{
		uint32_t color = ((0x00 << 24) | (uint32_t) _r << 16 | (uint32_t) _g << 8 | (uint32_t) _b);
		return color;
	}

	/**************************************************/
	static inline uint32_t palette12(const int _index)
	{
		return colorPalette12[_index % 12];
	}

	/**************************************************/
	static inline uint32_t palette35(const int _index)
	{
		return colorPalette35[_index % 35];
	}

	/**************************************************/
	static std::pair<Eigen::Vector3f, Eigen::Vector3f> generatePerpendicularPointsInPlane(const Eigen::Hyperplane<float, 3> &plane_,
			const Eigen::Vector3f &point_);

	/**************************************************/
	static inline double getSSE(const cv::Mat &_vectors,
								const cv::Mat &centers_,
								const cv::Mat &labels_)
	{
		double sse = 0;
		for (int i = 0; i < _vectors.rows; i++)
		{
			float norm = cv::norm(_vectors.row(i), centers_.row(labels_.at<int>(i)));
			sse += (norm * norm);
		}

		return sse;
	}

	/**************************************************/
	template<typename T> static inline int sign(T val)
	{
		return (T(0) < val) - (val < T(0));
	}

	/**************************************************/
	template<class T> static inline double angle(const T &vector1_,
			const T &vector2_)
	{
		return atan2(vector1_.cross(vector2_).norm(), vector1_.dot(vector2_));
	}

	/**************************************************/
	template<class T> static inline double signedAngle(const T &vector1_,
			const T &vector2_,
			const T &_normal)
	{
		double direction = _normal.dot(vector1_.cross(vector2_));

		// Check if the cross product is not zero
		if (fabs(direction) > 1E-7)
		{
			if (direction < 0)
				return -atan2(vector1_.cross(vector2_).norm(), vector1_.dot(vector2_));
			else
				return atan2(vector1_.cross(vector2_).norm(), vector1_.dot(vector2_));
		}
		else
		{
			if (vector1_.dot(vector2_) >= 0)
				return 0;
			else
				return M_PI;
		}
	}

	/**************************************************/
	static SynCloudType getSynCloudType(const std::string &_type);

	/**************************************************/
	static SequenceStat getStatType(const std::string &_type);

	/**************************************************/
	static ClusteringImplementation getClusteringImplementation(const std::string &_type);

	/**************************************************/
	static MetricType getMetricType(const std::string &_type);

private:
	Utils();
	~Utils();
};
