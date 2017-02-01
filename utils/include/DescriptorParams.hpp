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
namespace Params
{
enum DescriptorType
{
	DESCRIPTOR_UNKNOWN,
	DESCRIPTOR_DCH,
	DESCRIPTOR_SHOT,
	DESCRIPTOR_USC,
	DESCRIPTOR_PFH,
	DESCRIPTOR_FPFH,
	DESCRIPTOR_ROPS,
	DESCRIPTOR_SPIN_IMAGE,
};
static std::string descType[] = {
	BOOST_STRINGIZE(DESCRIPTOR_UNKNOWN),
	BOOST_STRINGIZE(DESCRIPTOR_DCH),
	BOOST_STRINGIZE(DESCRIPTOR_SHOT),
	BOOST_STRINGIZE(DESCRIPTOR_USC),
	BOOST_STRINGIZE(DESCRIPTOR_PFH),
	BOOST_STRINGIZE(DESCRIPTOR_FPFH),
	BOOST_STRINGIZE(DESCRIPTOR_ROPS),
	BOOST_STRINGIZE(DESCRIPTOR_SPIN_IMAGE)
};
}


/**************************************************/
/**************************************************/
struct DescriptorParams;
typedef boost::shared_ptr<DescriptorParams> DescriptorParamsPtr;

struct DescriptorParams
{
	Params::DescriptorType type; //  Descriptor type

	DescriptorParams()
	{
		type = Params::DESCRIPTOR_UNKNOWN;
	}

	/**************************************************/
	virtual void load(const YAML::Node &config_) = 0;

	/**************************************************/
	virtual std::string toString() const = 0;

	/**************************************************/
	virtual YAML::Node toNode() const = 0;

	/**************************************************/
	static Params::DescriptorType toType(const std::string &type_);

	/**************************************************/
	static DescriptorParamsPtr create(const Params::DescriptorType type_);
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
	int binNumber; // Number of bins per band
	Params::Statistic stat; // Statistic used in the descriptor

	float angle; // Orientation of the zero band (run time parameter)


	/**************************************************/
	DCHParams()
	{
		type = Params::DESCRIPTOR_DCH;
		searchRadius = 0.05;
		bandNumber = 4;
		bandWidth = 0.01;
		bidirectional = true;
		useProjection = true;
		binNumber = 1;
		stat = Params::STAT_MEAN;

		angle  = 0;
	}

	/**************************************************/
	void load(const YAML::Node &config_);

	/**************************************************/
	std::string toString() const;

	/**************************************************/
	YAML::Node toNode() const;

	/**************************************************/
	int sizePerBand() const;

	/**************************************************/
	float binSize() const;

	/**************************************************/
	float bandsAngleRange() const;

	/**************************************************/
	float bandsAngleStep() const;
};


/**************************************************/
/**************************************************/
struct SHOTParams: public DescriptorParams
{
	float searchRadius;

	SHOTParams()
	{
		type = Params::DESCRIPTOR_SHOT;
		searchRadius = 0.03;
	}

	/**************************************************/
	void load(const YAML::Node &config_);

	/**************************************************/
	std::string toString() const;

	/**************************************************/
	YAML::Node toNode() const;
};


/**************************************************/
/**************************************************/
struct USCParams: public DescriptorParams
{
	float searchRadius;

	USCParams()
	{
		type = Params::DESCRIPTOR_USC;
		searchRadius = 0.03;
	}

	/**************************************************/
	void load(const YAML::Node &config_);

	/**************************************************/
	std::string toString() const;

	/**************************************************/
	YAML::Node toNode() const;
};


/**************************************************/
/**************************************************/
struct PFHParams: public DescriptorParams
{
	float searchRadius;

	PFHParams()
	{
		type = Params::DESCRIPTOR_PFH;
		searchRadius = 0.03;
	}

	/**************************************************/
	void load(const YAML::Node &config_);

	/**************************************************/
	std::string toString() const;

	/**************************************************/
	YAML::Node toNode() const;
};


/**************************************************/
/**************************************************/
struct FPFHParams: public DescriptorParams
{
	float searchRadius;

	FPFHParams()
	{
		type = Params::DESCRIPTOR_FPFH;
		searchRadius = 0.03;
	}

	/**************************************************/
	void load(const YAML::Node &config_);

	/**************************************************/
	std::string toString() const;

	/**************************************************/
	YAML::Node toNode() const;
};


/**************************************************/
/**************************************************/
struct ROPSParams: public DescriptorParams
{
	float searchRadius;
	int partitionsNumber;
	int rotationsNumber;
	float supportRadius;

	ROPSParams()
	{
		type = Params::DESCRIPTOR_ROPS;
		searchRadius = 0.03;
	}

	/**************************************************/
	void load(const YAML::Node &config_);

	/**************************************************/
	std::string toString() const;

	/**************************************************/
	YAML::Node toNode() const;
};


/**************************************************/
/**************************************************/
struct SpinImageParams: public DescriptorParams
{
	float searchRadius;
	int imageWidth;

	SpinImageParams()
	{
		type = Params::DESCRIPTOR_SPIN_IMAGE;
		searchRadius = 0.03;
		imageWidth = 8;
	}

	/**************************************************/
	void load(const YAML::Node &config_);

	/**************************************************/
	std::string toString() const;

	/**************************************************/
	YAML::Node toNode() const;
};