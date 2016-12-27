/**
 * Author: rodrigo
 * 2016
 */
#include "DescriptorParams.hpp"
#include <plog/Log.h>
#include <pcl/pcl_macros.h>


Params::DescriptorType DescriptorParams::toType(const std::string &type_)
{
	if (boost::iequals(type_, "DCH") || boost::iequals(type_, Params::descType[Params::DESCRIPTOR_DCH]))
		return Params::DESCRIPTOR_DCH;
	else if (boost::iequals(type_, "SHOT") || boost::iequals(type_, Params::descType[Params::DESCRIPTOR_SHOT]))
		return Params::DESCRIPTOR_SHOT;
	else if (boost::iequals(type_, "USC") || boost::iequals(type_, Params::descType[Params::DESCRIPTOR_USC]))
		return Params::DESCRIPTOR_USC;
	else if (boost::iequals(type_, "PFH") || boost::iequals(type_, Params::descType[Params::DESCRIPTOR_PFH]))
		return Params::DESCRIPTOR_PFH;
	else if (boost::iequals(type_, "ROPS") || boost::iequals(type_, Params::descType[Params::DESCRIPTOR_ROPS]))
		return Params::DESCRIPTOR_ROPS;

	LOGW << "Wrong descriptor type, assuming DCH";
	return Params::DESCRIPTOR_DCH;
}

DescriptorParamsPtr DescriptorParams::create(const Params::DescriptorType type_)
{
	switch (type_)
	{
	default:
	case Params::DESCRIPTOR_UNKNOWN:
		LOGW << "Bad descriptor type for params instantiation, assuming DCH";

	case Params::DESCRIPTOR_DCH:
		return DescriptorParamsPtr(new DCHParams());

	case Params::DESCRIPTOR_SHOT:
		return DescriptorParamsPtr(new SHOTParams());

	case Params::DESCRIPTOR_USC:
		return DescriptorParamsPtr();

	case Params::DESCRIPTOR_PFH:
		return DescriptorParamsPtr();

	case Params::DESCRIPTOR_ROPS:
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
	stat = Params::toStatType(config_["stat"].as<std::string>());
}

std::string DCHParams::toString() const
{
	std::stringstream stream;
	stream << std::boolalpha
		   << "type:" << Params::descType[type]
		   << " searchRadius:" << searchRadius
		   << " bandNumber:" << bandNumber
		   << " bandWidth:" << bandWidth
		   << " bidirectional:" << bidirectional
		   << " useProjection:" << useProjection
		   << " sequenceBin:" << sequenceBin
		   << " stat:" << Params::stat[stat];
	return stream.str();
}

YAML::Node DCHParams::toNode() const
{
	std::string sType = Params::descType[type].substr(11);
	std::string statString = Params::toString(stat);

	YAML::Node node;
	node["type"] = sType;
	node[sType]["searchRadius"] = searchRadius;
	node[sType]["bandNumber"] = bandNumber;
	node[sType]["bandWidth"] = bandWidth;
	node[sType]["bidirectional"] = bidirectional;
	node[sType]["useProjection"] = useProjection;
	node[sType]["sequenceBin"] = sequenceBin;
	node[sType]["stat"] = statString;

	return node;
}

int DCHParams::sizePerBand() const
{
	switch (stat)
	{
	default:
	case Params::STAT_MEAN:
	case Params::STAT_MEDIAN:
		return (bidirectional ? searchRadius * 2.0 : searchRadius) / sequenceBin;

	case Params::STAT_HISTOGRAM_10:
		return ceil(M_PI / DEG2RAD(10));

	case Params::STAT_HISTOGRAM_20:
		return ceil(M_PI / DEG2RAD(20));

	case Params::STAT_HISTOGRAM_30:
		return ceil(M_PI / DEG2RAD(30));

	case Params::STAT_HISTOGRAM_BIN_10:
		return ceil(M_PI / DEG2RAD(10)) * 2;

	case Params::STAT_HISTOGRAM_BIN_20:
		return ceil(M_PI / DEG2RAD(20)) * 2;

	case Params::STAT_HISTOGRAM_BIN_30:
		return ceil(M_PI / DEG2RAD(30)) * 2;
	}
}

float DCHParams::bandsAngleRange() const
{
	if (bidirectional)
		return M_PI;
	else
		return 2 * M_PI;
}

float DCHParams::bandsAngleStep() const
{
	return bandsAngleRange() / bandNumber;
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
		   << "type:" << Params::descType[type]
		   << " searchRadius:" << searchRadius;
	return stream.str();
}

YAML::Node SHOTParams::toNode() const
{
	std::string sType = Params::descType[type].substr(11);

	YAML::Node node;
	node["type"] = sType;
	node[sType]["searchRadius"] = searchRadius;

	return node;
}
