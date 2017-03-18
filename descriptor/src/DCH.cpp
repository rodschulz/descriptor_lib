/**
 * Author: rodrigo
 * 2015
 */
#include "DCH.hpp"
#include <pcl/io/pcd_io.h>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <boost/accumulators/statistics/count.hpp>
#include <plog/Log.h>
#include "Config.hpp"
#include "CloudFactory.hpp"
#include "PointFactory.hpp"


using namespace boost::accumulators;


void DEBUG_generateAxes(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_,
						const std::vector<BandPtr> &bands_,
						const int target_,
						const std::string &debugId_,
						const DCHParams *params_)
{
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr axesCloud = CloudFactory::createColorCloud(cloud_, Utils::palette12(0));


	pcl::PointCloud<pcl::PointNormal>::Ptr patch = Extractor::getNeighbors(cloud_, cloud_->at(target_), params_->searchRadius);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr patchCloud = CloudFactory::createColorCloud(patch, 255, 0, 0);
	*axesCloud += *patchCloud;


	for (size_t b = 0; b < bands_.size(); b++)
	{
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr lineCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
		for (float t = (params_->bidirectional ? -params_->searchRadius : 0);
				t <= params_->searchRadius;
				t += params_->searchRadius / 50)
		{
			Eigen::Vector3f point = bands_[b]->axis.pointAt(t);
			lineCloud->push_back(PointFactory::createPointXYZRGBNormal(point.x(), point.y(), point.z(), 0, 0, 0, 0, (PointColor)Utils::palette12(b + 1)));
		}

		*axesCloud += *lineCloud;
	}

	pcl::io::savePCDFileASCII(DEBUG_DIR DEBUG_PREFIX + debugId_ + CLOUD_FILE_EXTENSION, *axesCloud);
}

std::vector<BandPtr> DCH::calculateDescriptor(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_,
		const DescriptorParamsPtr &params_,
		const int targetPointIndex_)
{
	pcl::PointNormal target = cloud_->at(targetPointIndex_);
	return calculateDescriptor(cloud_, params_, target);
}

std::vector<BandPtr> DCH::calculateDescriptor(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_,
		const DescriptorParamsPtr &params_,
		const pcl::PointNormal &target_)
{
	DCHParams *params = dynamic_cast<DCHParams *>(params_.get());

	// Get target point and surface patch
	pcl::PointCloud<pcl::PointNormal>::Ptr patch = Extractor::getNeighbors(cloud_, target_, params->searchRadius);

	// Extract bands
	std::vector<BandPtr> bands = Extractor::getBands(patch, target_, params);
	DCH::fillDescriptor(bands, params_);

	return bands;
}

void DCH::computeDense(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_,
					   const DescriptorParamsPtr &params_,
					   cv::Mat &descriptors_)
{
	DCHParams *params = dynamic_cast<DCHParams *>(params_.get());
	int bandSize = params->sizePerBand();

	// Resize the matrix in case it doesn't match the required dimensions
	int rows = cloud_->size();
	int cols = bandSize * params->bandNumber;
	if (descriptors_.rows != rows || descriptors_.cols != cols)
		descriptors_ = cv::Mat::zeros(rows, cols, CV_32FC1);

	// Extract the descriptors
	for (size_t i = 0; i < cloud_->size(); i++)
	{
		std::vector<BandPtr> bands = DCH::calculateDescriptor(cloud_, params_, cloud_->points[i]);

		for (size_t j = 0; j < bands.size(); j++)
			memcpy(&descriptors_.at<float>(i, j * bandSize), &bands[j]->descriptor[0], sizeof(float) * bandSize);
	}
}

void DCH::computePoint(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_,
					   const DescriptorParamsPtr &params_,
					   const int target_,
					   Eigen::VectorXf &descriptor_,
					   const std::string &debugId_)
{
	DCHParams *params = dynamic_cast<DCHParams *>(params_.get());
	int bandSize = params->sizePerBand();

	std::vector<BandPtr> bands = DCH::calculateDescriptor(cloud_, params_, target_);
	descriptor_.resize(bandSize * bands.size(), 1);

	for (size_t j = 0; j < bands.size(); j++)
		for (size_t k = 0; k < bands[j]->descriptor.size(); k++)
			descriptor_(j * bandSize + k) = bands[j]->descriptor[k];


	if (Config::debugEnabled())
		DEBUG_generateAxes(cloud_, bands, target_, debugId_, params);
}

std::vector<Histogram>
DCH::generateAngleHistograms(const std::vector<BandPtr> &bands_,
							 const bool useProjection_)
{
	// TODO move this method to the output class, since this is only to generate the histogram generated as output

	std::vector<Histogram> histograms = std::vector<Histogram>();
	histograms.reserve(bands_.size());

	Eigen::Vector3f targetNormal = bands_[0]->origin.getNormalVector3fMap();
	for (size_t i = 0; i < bands_.size(); i++)
	{
		BandPtr band = bands_[i];
		histograms.push_back(Histogram(ANGLE));

		for (size_t j = 0; j < band->points->size(); j++)
		{
			Eigen::Vector3f normal = band->points->points[j].getNormalVector3fMap();
			histograms.back().add(calculateAngle(targetNormal, normal, band->plane, useProjection_));
		}
	}

	return histograms;
}

void DCH::fillDescriptor(std::vector<BandPtr> &bands_,
						 const DescriptorParamsPtr &params_)
{
	DCHParams *params = dynamic_cast<DCHParams *>(params_.get());
	if (params == NULL)
	{
		LOGE << "Wrong parameters type (DCH::fillDescriptor)";
		throw std::runtime_error("Unable to cast the given parameters");
	}

	LOGD << "Filling " << Params::stat[params->stat];


	switch (params->stat)
	{
	default:
	case Params::STAT_MEAN:
	case Params::STAT_MEDIAN:
	{
		float binSize = params->binSize();
		for (size_t i = 0; i < bands_.size(); i++)
		{
			BandPtr band = bands_[i];

			Eigen::Vector3f pointNormal = band->origin.getNormalVector3fMap();
			Eigen::Vector3f planeNormal = band->plane.normal();
			Eigen::Vector3f n = planeNormal.cross(pointNormal).normalized();
			Eigen::Hyperplane<float, 3> plane = Eigen::Hyperplane<float, 3>(n, band->origin.getVector3fMap());


			// Accumulate
			std::map<int, accumulator_set<double, features<tag::mean, tag::median, tag::min> > > dataMap;
			for (size_t j = 0; j < band->points->size(); j++)
			{
				pcl::PointNormal p = band->points->at(j);
				double theta = calculateAngle(pointNormal, (Eigen::Vector3f) p.getNormalVector3fMap(), band->plane, params->useProjection);
				int index = plane.signedDistance((Eigen::Vector3f) p.getVector3fMap()) / binSize;

				if (dataMap.find(index) == dataMap.end())
					dataMap[index] = accumulator_set<double, features<tag::mean, tag::median, tag::min> >();

				dataMap[index](theta);
			}


			// Fill the descriptor
			for (int j = 0; j < params->sizePerBand(); j++)
			{
				if (dataMap.find(j) != dataMap.end())
				{
					float value = params->stat == Params::STAT_MEAN
								  ? (float) mean(dataMap[j])
								  : (float) median(dataMap[j]);
					band->descriptor.push_back(value);
				}
				else
					band->descriptor.push_back(5);
			}
		}
	}
	break;

	case Params::STAT_HISTOGRAM_10:
	case Params::STAT_HISTOGRAM_20:
	case Params::STAT_HISTOGRAM_30:
	{
		float angleStep = params->stat == Params::STAT_HISTOGRAM_10 ? 10 :
						  (params->stat == Params::STAT_HISTOGRAM_20 ? 20 : 30);


		// compute the total number of points and the ratio per band
		// float totalPoints = 0;
		// for (size_t i = 0; i < bands_.size(); i++)
		// 	totalPoints += bands_[i]->points->size();
		// std::vector<float> ratio;
		// for (size_t i = 0; i < bands_.size(); i++)
		// 	ratio.push_back(bands_[i]->points->size() / totalPoints);


		// compute the histograms
		std::vector<Histogram> histograms = generateAngleHistograms(bands_, params->useProjection);
		for (size_t i = 0; i < histograms.size(); i++)
		{
			Bins b = histograms[i].getBins(DEG2RAD(angleStep), -M_PI / 2, M_PI / 2);
			bands_[i]->descriptor.clear();
			bands_[i]->descriptor.insert(bands_[i]->descriptor.begin(), b.bins.begin(), b.bins.end());

			// // scale the band's descriptor according to the computed ratios
			// for (size_t j = 0; j < bands_[i]->descriptor.size(); j++)
			// 	bands_[i]->descriptor[j] *= ratio[i];
		}
	}
	break;

	case Params::STAT_HISTOGRAM_BIN_10:
	case Params::STAT_HISTOGRAM_BIN_20:
	case Params::STAT_HISTOGRAM_BIN_30:
	{
		throw std::runtime_error("Not yet implemented (DCH::fillDescriptor)");
	}
	break;
	}
}
