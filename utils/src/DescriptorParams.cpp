/**
 * Author: rodrigo
 * 2016
 */
#include "DescriptorParams.hpp"
#include <plog/Log.h>


DescriptorType DescriptorParams::toType(const std::string &type_)
{
	if (boost::iequals(type_, "DCH") || boost::iequals(type_, descType[DESCRIPTOR_DCH]))
		return DESCRIPTOR_DCH;
	else if (boost::iequals(type_, "SHOT") || boost::iequals(type_, descType[DESCRIPTOR_SHOT]))
		return DESCRIPTOR_SHOT;
	else if (boost::iequals(type_, "USC") || boost::iequals(type_, descType[DESCRIPTOR_USC]))
		return DESCRIPTOR_USC;
	else if (boost::iequals(type_, "PFH") || boost::iequals(type_, descType[DESCRIPTOR_PFH]))
		return DESCRIPTOR_PFH;
	else if (boost::iequals(type_, "ROPS") || boost::iequals(type_, descType[DESCRIPTOR_ROPS]))
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

YAML::Node DCHParams::toNode() const
{
	std::string sType = descType[type].substr(11);
	std::string stat;
	switch (sequenceStat)
	{
	default:
	case STAT_MEAN:
		stat = "mean";
	case STAT_MEDIAN:
		stat = "median";
	case STAT_HISTOGRAM:
		stat = "histogram";
	}

	YAML::Node node;
	node["type"] = sType;
	node[sType]["searchRadius"] = searchRadius;
	node[sType]["bandNumber"] = bandNumber;
	node[sType]["bandWidth"] = bandWidth;
	node[sType]["bidirectional"] = bidirectional;
	node[sType]["useProjection"] = useProjection;
	node[sType]["sequenceBin"] = sequenceBin;
	node[sType]["sequenceStat"] = stat;

	return node;
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

YAML::Node SHOTParams::toNode() const
{
	std::string sType = descType[type].substr(11);

	YAML::Node node;
	node["type"] = sType;
	node[sType]["searchRadius"] = searchRadius;

	return node;
}
