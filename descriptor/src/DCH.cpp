/**
 * Author: rodrigo
 * 2016
 */
#include "DCH.hpp"
#include "Utils.hpp"


void DCH::load(const YAML::Node &config_)
{
	searchRadius = config_["searchRadius"].as<float>();
	bandNumber = config_["bandNumber"].as<int>();
	bandWidth = config_["bandWidth"].as<float>();
	bidirectional = config_["bidirectional"].as<bool>();
	useProjection = config_["useProjection"].as<bool>();
	binNumber = config_["binNumber"].as<int>();
	sequenceStat = Utils::getStatType(config_["sequenceStat"].as<std::string>());
}

std::string DCH::toString() const
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

void DCH::computeDense(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_,
					   cv::Mat &descriptors_) const
{
}
