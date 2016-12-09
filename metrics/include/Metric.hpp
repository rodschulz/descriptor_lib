/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <opencv2/core/core.hpp>
#include <boost/shared_ptr.hpp>

enum MetricType
{
	METRIC_EUCLIDEAN, METRIC_CLOSEST_PERMUTATION, METRIC_CLOSEST_PERMUTATION_WITH_CONFIDENCE
};
static std::string metricType[] = { BOOST_STRINGIZE(METRIC_EUCLIDEAN), BOOST_STRINGIZE(METRIC_CLOSEST_PERMUTATION), BOOST_STRINGIZE(METRIC_CLOSEST_PERMUTATION_WITH_CONFIDENCE) };

// Metric class definition
class Metric
{
public:
	// Returns the distance between the given vectors, according to the current metric
	virtual double distance(const cv::Mat &_vector1, const cv::Mat &_vector2) const = 0;

	// Returns the mean point amongst the given items, according to the given labels
	virtual cv::Mat calculateMeans(const int clusterNumber_, const cv::Mat &items_, const cv::Mat &labels_, const cv::Mat &currentMeans_ = cv::Mat()) const = 0;

	// Returns the medoid point of the given items, according to the given labels
//	virtual cv::Mat calculateMedoids(const int clusterNumber_, const cv::Mat &items_, const cv::Mat &labels_, const cv::Mat &currentMedoids = cv::Mat());

	// Returns the type of the current metric
	virtual MetricType getType() const = 0;

	// Returns the parameters used to construct the current instance
	virtual std::vector<std::string> getConstructionParams() const = 0;

	// Validates and fixes the given means, according to the metric's definition
	virtual void validateMeans(cv::Mat &means_) const = 0;

	// Enables/disabled the debug generation
	void setDebug(const bool &status_)
	{
		debugEnabled = status_;
	}

protected:
	// Constructor
	Metric()
	{
		debugEnabled = false;
	}
	// Destructor
	virtual ~Metric()
	{
	}

	// Debug generation flag
	bool debugEnabled;
};

// Metric's shared pointer
typedef boost::shared_ptr<Metric> MetricPtr;
