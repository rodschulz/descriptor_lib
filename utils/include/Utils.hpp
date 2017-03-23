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
#include "DescriptorParams.hpp"


// Colors defined to be used for points
typedef enum PointColor
{
	COLOR_FIREBRICK 		= 0xB22222,
	COLOR_RED 				= 0xFF0000,
	COLOR_SALMON 			= 0xFA8072,
	COLOR_ORANGE 			= 0xFFA500,
	COLOR_GOLDEN_ROD 		= 0xDAA520,
	COLOR_OLIVE 			= 0x808000,
	COLOR_KHAKI 			= 0xF0E68C,
	COLOR_YELLOW 			= 0xFFFF00,
	COLOR_DARK_GREEN 		= 0x006400,
	COLOR_GREEN 			= 0x008000,
	COLOR_LIME 				= 0x00FF00,
	COLOR_LIGHT_GREEN 		= 0x90EE90,
	COLOR_DARK_SEA_GREEN 	= 0x8FBC8F,
	COLOR_MEDIUM_AQUA 		= 0x66CDAA,
	COLOR_TEAL 				= 0x008080,
	COLOR_CYAN 				= 0x00FFFF,
	COLOR_TURQUOISE 		= 0x40E0D0,
	COLOR_DODGER_BLUE 		= 0x1E90FF,
	COLOR_SKY_BLUE 			= 0x87CEEB,
	COLOR_NAVY 				= 0x000080,
	COLOR_BLUE 				= 0x0000FF,
	COLOR_BLUE_VIOLET 		= 0x8A2BE2,
	COLOR_SLATE_BLUE 		= 0x6A5ACD,
	COLOR_DARK_MAGENTA 		= 0x8B008B,
	COLOR_PLUM 				= 0xDDA0DD,
	COLOR_MAGENTA 			= 0xFF00FF,
	COLOR_DEEP_PINK 		= 0xFF1493,
	COLOR_HOT_PINK 			= 0xFF69B4,
	COLOR_BEIGE 			= 0xF5F5DC,
	COLOR_BROWN 			= 0xA0522D,
	COLOR_ROSY_BROWN 		= 0xBC8F8F,
	COLOR_SLATE_GRAY 		= 0x708090,
	COLOR_BLACK 			= 0x000000,
	COLOR_GRAY 				= 0x808080,
	COLOR_SILVER 			= 0xC0C0C0,
	COLOR_WHITE 			= 0xFFFFFF,

	//Color brewer definitions
	COLOR_BREWER_QUALITATIVE_SKYBLUE 		= 0xA6CEE3,
	COLOR_BREWER_QUALITATIVE_BLUE 			= 0x1F78B4,
	COLOR_BREWER_QUALITATIVE_LIGHTGREEN 	= 0xB2DF8A,
	COLOR_BREWER_QUALITATIVE_GREEN 			= 0x33A02C,
	COLOR_BREWER_QUALITATIVE_PINK 			= 0xFB9A99,
	COLOR_BREWER_QUALITATIVE_RED 			= 0xE31A1C,
	COLOR_BREWER_QUALITATIVE_SALMON 		= 0xFDBF6F,
	COLOR_BREWER_QUALITATIVE_ORANGE 		= 0xFF7F00,
	COLOR_BREWER_QUALITATIVE_LAVANDA 		= 0xCAB2D6,
	COLOR_BREWER_QUALITATIVE_PURPLE 		= 0x6A3D9A,
	COLOR_BREWER_QUALITATIVE_LIGHTYELLOW 	= 0xFFFF99,
	COLOR_BREWER_QUALITATIVE_BROWN 			= 0xB15928,
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
	static plog::Severity getLogLevel(const std::string &filename_);

	/**************************************************/
	static void cleanDirectories(const std::string &workingDir_);

	/**************************************************/
	static std::string getWorkingDirectory();

	/**************************************************/
	static std::string getCalculationConfigHash(const std::string inputCloudFile_,
			const double normalEstimationRadius_,
			const DescriptorParamsPtr &descriptorParams_,
			const CloudSmoothingParams &smoothingParams_);

	/**************************************************/
	static std::string getFileChecksum(const std::string filename_);

	/**************************************************/
	static int getRandomNumber(const int min_,
							   const int max_);

	/**************************************************/
	static std::vector<float> getRandomRealArray(const unsigned int size_,
			const float min_,
			const float max_,
			const bool allowRepetition_);

	/**************************************************/
	static std::vector<int> getRandomIntArray(const unsigned int size_,
			const int min_,
			const int max_,
			const bool allowRepetition_);

	/**************************************************/
	static std::string num2Hex(const size_t number_);

	/**************************************************/
	static inline uint32_t getColor(const uint8_t r_,
									const uint8_t g_,
									const uint8_t b_)
	{
		uint32_t color = ((0x00 << 24) | (uint32_t) r_ << 16 | (uint32_t) g_ << 8 | (uint32_t) b_);
		return color;
	}

	/**************************************************/
	static inline uint32_t getColor(const PointColor &color_)
	{
		uint8_t r = color_ & 0x00FF0000;
		uint8_t g = color_ & 0x0000FF00;
		uint8_t b = color_ & 0x000000FF;
		uint32_t color = ((0x00 << 24) | (uint32_t) r << 16 | (uint32_t) g << 8 | (uint32_t) b);
		return color;
	}

	/**************************************************/
	static inline uint32_t palette12(const int index_)
	{
		return colorPalette12[index_ % 12];
	}

	/**************************************************/
	static inline uint32_t palette35(const int index_)
	{
		return colorPalette35[index_ % 35];
	}

	/**************************************************/
	static std::pair<Eigen::Vector3f, Eigen::Vector3f> generateAxes(const Eigen::Hyperplane<float, 3> &plane_,
			const Eigen::Vector3f &point_);

	/**************************************************/
	static inline double getSSE(const cv::Mat &vectors_,
								const cv::Mat &centers_,
								const cv::Mat &labels_)
	{
		double sse = 0;
		for (int i = 0; i < vectors_.rows; i++)
		{
			float norm = cv::norm(vectors_.row(i), centers_.row(labels_.at<int>(i)));
			sse += (norm * norm);
		}

		return sse;
	}

	/**************************************************/
	template<typename T>
	static inline int sign(T val)
	{
		return (T(0) < val) - (val < T(0));
	}

	/**************************************************/
	template<class T>
	static inline double angle(const T &vector1_,
							   const T &vector2_)
	{
		return atan2(vector1_.cross(vector2_).norm(), vector1_.dot(vector2_));
	}

	/**************************************************/
	template<class T>
	static inline double signedAngle(const T &vector1_,
									 const T &vector2_,
									 const T &normal_)
	{
		double direction = normal_.dot(vector1_.cross(vector2_));

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

private:
	Utils();
	~Utils();
};
