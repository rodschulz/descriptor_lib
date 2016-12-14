/**
 * Author: rodrigo
 * 2015
 */
#pragma once

#include <string>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include "ExecutionParams.hpp"


/**************************************************/
/**************************************************/
enum DescriptorType
{
	DESCRIPTOR_UNKNOWN,
	DESCRIPTOR_DCH,
	DESCRIPTOR_SHOT,
	DESCRIPTOR_USC,
	DESCRIPTOR_PFH,
	DESCRIPTOR_ROPS,
};
static std::string descType[] = {
	BOOST_STRINGIZE(DESCRIPTOR_UNKNOWN),
	BOOST_STRINGIZE(DESCRIPTOR_DCH),
	BOOST_STRINGIZE(DESCRIPTOR_SHOT),
	BOOST_STRINGIZE(DESCRIPTOR_USC),
	BOOST_STRINGIZE(DESCRIPTOR_PFH),
	BOOST_STRINGIZE(DESCRIPTOR_ROPS)
};


/**************************************************/
/**************************************************/
struct DescriptorParams;
typedef boost::shared_ptr<DescriptorParams> DescriptorParamsPtr;

struct DescriptorParams
{
	DescriptorType type; //  Descriptor type

	DescriptorParams()
	{
		type = DESCRIPTOR_UNKNOWN;
	}

	/**************************************************/
	virtual void load(const YAML::Node &config_) = 0;

	/**************************************************/
	virtual std::string toString() const = 0;

	/**************************************************/
	static DescriptorType toType(const std::string &type_)
	{
		if (boost::iequals(type_, "DCH"))
			return DESCRIPTOR_DCH;
		else if (boost::iequals(type_, "SHOT"))
			return DESCRIPTOR_SHOT;
		else if (boost::iequals(type_, "USC"))
			return DESCRIPTOR_USC;
		else if (boost::iequals(type_, "PFH"))
			return DESCRIPTOR_PFH;
		else if (boost::iequals(type_, "ROPS"))
			return DESCRIPTOR_ROPS;

		std::cout << "WARNING: wrong descriptor type, assuming DCH";
		return DESCRIPTOR_DCH;
	}

	/**************************************************/
	static DescriptorParamsPtr create(const DescriptorType type_);
};


/**************************************************/
/**************************************************/
struct DCHParams: public DescriptorParams
{
	double searchRadius; // Search radius for the KNN search method
	int bandNumber; // Number of bands of the descriptor
	double bandWidth; // Width of each band
	bool bidirectional; // True if each band is bidirectional
	bool useProjection; // True if the angle calculation is using a projection
	double sequenceBin; // Size of the bins used in the sequence construction
	SequenceStat sequenceStat; // Statistic used in the descriptor

	/**************************************************/
	DCHParams()
	{
		type = DESCRIPTOR_DCH;
		searchRadius = 0.05;
		bandNumber = 4;
		bandWidth = 0.01;
		bidirectional = true;
		useProjection = true;
		sequenceBin = 0.01;
		sequenceStat = STAT_MEAN;
	}

	/**************************************************/
	void load(const YAML::Node &config_)
	{
		searchRadius = config_["searchRadius"].as<float>();
		bandNumber = config_["bandNumber"].as<int>();
		bandWidth = config_["bandWidth"].as<float>();
		bidirectional = config_["bidirectional"].as<bool>();
		useProjection = config_["useProjection"].as<bool>();
		sequenceBin = config_["sequenceBin"].as<float>();
		sequenceStat = toStatType(config_["sequenceStat"].as<std::string>());
	}

	/**************************************************/
	std::string toString() const
	{
		std::stringstream stream;
		stream << std::boolalpha
			   << "type:" << descType[type]
			   << " searchRadius:" << searchRadius
			   << " bandNumber:" << bandNumber
			   << " bandWidth:" << bandWidth
			   << " bidirectional:" << bidirectional
			   << " useProjection:" << useProjection
			   << " sequenceBin:" << sequenceBin
			   << " sequenceStat:" << seqStat[sequenceStat];
		return stream.str();
	}

	/**************************************************/
	int getSequenceLength() const
	{
		return (bidirectional ? searchRadius * 2.0 : searchRadius) / sequenceBin;
	}

	/**************************************************/
	double getBandsAngularRange() const
	{
		if (bidirectional)
			return M_PI;
		else
			return 2 * M_PI;
	}

	/**************************************************/
	double getBandsAngularStep() const
	{
		return getBandsAngularRange() / bandNumber;
	}
};


/**************************************************/
/**************************************************/
struct SHOTParams: public DescriptorParams
{
	float searchRadius;

	SHOTParams()
	{
		type = DESCRIPTOR_SHOT;
		searchRadius = 0.03;
	}

	/**************************************************/
	void load(const YAML::Node &config_)
	{
		searchRadius = config_["searchRadius"].as<float>();
	}

	/**************************************************/
	std::string toString() const
	{
		std::stringstream stream;
		stream << std::boolalpha
			   << "type:" << descType[type]
			   << " searchRadius:" << searchRadius;
		return stream.str();
	}
};


/**************************************************/
/**************************************************/
DescriptorParamsPtr DescriptorParams::create(const DescriptorType type_)
{
	switch (type_)
	{
	default:
	case DESCRIPTOR_UNKNOWN:
		std::cout << "WARNING: bad descriptor type for params instantiation, assuming DCH" << std::endl;

	case DESCRIPTOR_DCH:
		return DescriptorParamsPtr(new DCHParams());

	case DESCRIPTOR_SHOT:
		return DescriptorParamsPtr(new SHOTParams());

	case DESCRIPTOR_USC:
		return DescriptorParamsPtr();

	case DESCRIPTOR_PFH:
		return DescriptorParamsPtr();

	case DESCRIPTOR_ROPS:
		return DescriptorParamsPtr();
	}
}