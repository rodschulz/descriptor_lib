/**
 * Author: rodrigo
 * 2016
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
	virtual YAML::Node toNode() const = 0;

	/**************************************************/
	static DescriptorType toType(const std::string &type_);

	/**************************************************/
	static DescriptorParamsPtr create(const DescriptorType type_);
};


/**************************************************/
/**************************************************/
struct DCHParams: public DescriptorParams
{
	float searchRadius; // Search radius for the KNN search method
	int bandNumber; // Number of bands of the descriptor
	float bandWidth; // Width of each band
	bool bidirectional; // True if each band is bidirectional
	bool useProjection; // True if the angle calculation is using a projection
	float sequenceBin; // Size of the bins used in the sequence construction
	SequenceStat sequenceStat; // Statistic used in the descriptor

	float angle; // Orientation of the zero band (run time parameter)


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

		angle  = 0;
	}

	/**************************************************/
	void load(const YAML::Node &config_);

	/**************************************************/
	std::string toString() const;

	/**************************************************/
	YAML::Node toNode() const;

	/**************************************************/
	int getSequenceLength() const;

	/**************************************************/
	float getBandsAngularRange() const;

	/**************************************************/
	float getBandsAngularStep() const;
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
	void load(const YAML::Node &config_);

	/**************************************************/
	std::string toString() const;

	/**************************************************/
	YAML::Node toNode() const;
};
