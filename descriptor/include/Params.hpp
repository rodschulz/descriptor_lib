/**
 * Author: rodrigo
 * 2016
 */
#pragma once

#include <string>
#include <boost/shared_ptr.hpp>
#include "ExecutionParams.hpp"
#include "Config.hpp"
#include "Utils.hpp"


/**************************************************/
struct DescriptorParamsXX
{
	DescriptorParamsXX() {};
	virtual ~DescriptorParamsXX() {};

	virtual void load(const Config &config_) = 0;
	virtual std::string toString() const = 0;
};
typedef boost::shared_ptr<DescriptorParamsXX> DescriptorParamsPtr;


/**************************************************/
/********** DCH descriptor parameters *************/
/**************************************************/
struct DCHParams: public DescriptorParamsXX
{
	DCHParams()
	{
		searchRadius = 0;
		bandNumber = 4;
		bandWidth = 0.01;
		bidirectional = true;
		useProjection = true;
		binNumber = 0.01;
		sequenceStat = STAT_MEAN;
	};
	~DCHParams() {};

	/**************************************************/
	void load(const Config &config_)
	{
		YAML::Node descriptorConfig = config_.get()["descriptor"];

		searchRadius = descriptorConfig["searchRadius"].as<float>();
		bandNumber = descriptorConfig["bandNumber"].as<int>();
		bandWidth = descriptorConfig["bandWidth"].as<float>();
		bidirectional = descriptorConfig["bidirectional"].as<bool>();
		useProjection = descriptorConfig["useProjection"].as<bool>();
		binNumber = descriptorConfig["binNumber"].as<int>();
		sequenceStat = Utils::getStatType(descriptorConfig["sequenceStat"].as<std::string>());
	};

	/**************************************************/
	std::string toString() const
	{
		std::stringstream stream;
		stream << std::boolalpha
			   << "searchRadius:" << searchRadius
			   << " bandNumber:" << bandNumber
			   << " bandWidth:" << bandWidth
			   << " bidirectional:" << bidirectional
			   << " useProjection:" << useProjection
			   << " binNumber:" << binNumber
			   << " sequenceStat:" << seqStat[sequenceStat];
		return stream.str();
	}

	float searchRadius; // Extraction sphere's radius
	int bandNumber; // Number of bands to use in the descriptor
	float bandWidth; // Width of each descriptor's band
	bool bidirectional; // True if bands use the full diameter false otherwise
	bool useProjection; // True if angles are computed using a projection, false otherwise
	int binNumber; // Number of bins per band
	SequenceStat sequenceStat; // Statistic computed in each band
};


/**************************************************/
/********** SHOT descriptor parameters ************/
/**************************************************/
struct SHOTParams: public DescriptorParamsXX
{
	SHOTParams()
	{
		searchRadius = 0;
	};
	~SHOTParams() {};

	/**************************************************/
	void load(const Config &config_)
	{
		searchRadius = config_.get()["descriptor"]["searchRadius"].as<float>();
	};

	/**************************************************/
	std::string toString() const
	{
		std::stringstream stream;
		stream << "searchRadius:" << searchRadius;
		return stream.str();
	}

	float searchRadius;
};


/**************************************************/
/********** USC descriptor parameters *************/
/**************************************************/
// struct USCParams: public DescriptorParams
// {};


/**************************************************/
/********** PFH descriptor parameters *************/
/**************************************************/
// struct PFHParams: public DescriptorParams
// {};
