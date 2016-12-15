/**
 * Author: rodrigo
 * 2016
 */
#include "DescriptorParams.hpp"
#include <plog/Log.h>


DescriptorType DescriptorParams::toType(const std::string &type_)
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

	LOGW << "Wrong descriptor type, assuming DCH";
	return DESCRIPTOR_DCH;
}

DescriptorParamsPtr DescriptorParams::create(const DescriptorType type_)
{
	switch (type_)
	{
	default:
	case DESCRIPTOR_UNKNOWN:
		LOGW << "Bad descriptor type for params instantiation, assuming DCH";

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


/**************************************************/
/**************************************************/
void DCHParams::load(const YAML::Node &config_)
{
	searchRadius = config_["searchRadius"].as<float>();
	bandNumber = config_["bandNumber"].as<int>();
	bandWidth = config_["bandWidth"].as<float>();
	bidirectional = config_["bidirectional"].as<bool>();
	useProjection = config_["useProjection"].as<bool>();
	sequenceBin = config_["sequenceBin"].as<float>();
	sequenceStat = toStatType(config_["sequenceStat"].as<std::string>());
}

std::string DCHParams::toString() const
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

int DCHParams::getSequenceLength() const
{
	return (bidirectional ? searchRadius * 2.0 : searchRadius) / sequenceBin;
}

float DCHParams::getBandsAngularRange() const
{
	if (bidirectional)
		return M_PI;
	else
		return 2 * M_PI;
}

float DCHParams::getBandsAngularStep() const
{
	return getBandsAngularRange() / bandNumber;
}


/**************************************************/
/**************************************************/
void SHOTParams::load(const YAML::Node &config_)
{
	searchRadius = config_["searchRadius"].as<float>();
}

std::string SHOTParams::toString() const
{
	std::stringstream stream;
	stream << std::boolalpha
		   << "type:" << descType[type]
		   << " searchRadius:" << searchRadius;
	return stream.str();
}
