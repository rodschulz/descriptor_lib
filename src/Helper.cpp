/**
 * Author: rodrigo
 * 2015
 */
#include "Helper.h"
#include <pcl/filters/filter.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/surface/mls.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/common/io.h>
#include <ctype.h>
#include "CloudFactory.h"
#include "PointFactory.h"
#include "ExecutionParams.h"

Helper::Helper()
{
}

Helper::~Helper()
{
}

void Helper::removeNANs(pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud)
{
	std::vector<int> mapping;
	pcl::removeNaNFromPointCloud(*_cloud, *_cloud, mapping);
}

bool Helper::isNumber(const std::string &_str)
{
	bool number = true;
	for (size_t i = 0; i < _str.size(); i++)
	{
		number = number && isdigit(_str[i]);
	}
	return number;
}

std::string Helper::toHexString(const size_t _number)
{
	std::stringstream stream;
	stream << std::hex << _number;
	return stream.str();
}

pcl::PointCloud<pcl::Normal>::Ptr Helper::estimateNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud, const double _searchRadius)
{
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());

	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud(_cloud);

	if (_searchRadius > 0)
		normalEstimation.setRadiusSearch(_searchRadius);
	else
		normalEstimation.setKSearch(10);

	normalEstimation.setSearchMethod(kdtree);
	normalEstimation.compute(*normals);

	return normals;
}

bool Helper::loadCloud(pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const ExecutionParams &_params)
{
	bool loadOk = true;

	// Get cartesian data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ(new pcl::PointCloud<pcl::PointXYZ>());
	if (!_params.useSynthetic)
	{
		if (pcl::io::loadPCDFile<pcl::PointXYZ>(_params.inputLocation, *cloudXYZ) != 0)
		{
			std::cout << "ERROR: Can't read file from disk (" << _params.inputLocation << ")\n";
			loadOk = false;
		}

		switch (_params.smoothingType)
		{
			case SMOOTHING_GAUSSIAN:
				std::cout << "Applying gaussian smoothing\n";
				cloudXYZ = gaussianSmoothing(cloudXYZ, _params.gaussianSigma, _params.gaussianRadius);
				break;

			case SMOOTHING_MLS:
				std::cout << "Applying MLS smoothing\n";
				cloudXYZ = MLSSmoothing(cloudXYZ, _params.mlsRadius);
				break;
		}
	}
	else
	{
		switch (_params.synCloudType)
		{
			case CLOUD_CUBE:
				CloudFactory::generateCube(0.3, PointFactory::makePointXYZ(0.3, 0.3, 0.3), cloudXYZ);
				break;

			case CLOUD_CYLINDER:
				CloudFactory::generateCylinder(0.2, 0.5, PointFactory::makePointXYZ(0.4, 0.4, 0.4), cloudXYZ);
				break;

			case CLOUD_SPHERE:
				CloudFactory::generateSphere(0.2, PointFactory::makePointXYZ(0.2, 0.2, 0.2), cloudXYZ);
				break;

			default:
				cloudXYZ->clear();
				loadOk = false;
				std::cout << "WARNING, wrong cloud generation parameters\n";
		}
	}

	// Estimate normals
	if (loadOk)
	{
		Helper::removeNANs(cloudXYZ);
		pcl::PointCloud<pcl::Normal>::Ptr normals = Helper::estimateNormals(cloudXYZ, _params.normalEstimationRadius);

		_cloud->clear();
		concatenateFields(*cloudXYZ, *normals, *_cloud);
	}

	return loadOk;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Helper::gaussianSmoothing(const pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud, const double _sigma, const double _radius)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr smoothedCloud(new pcl::PointCloud<pcl::PointXYZ>());

	//Set up the Gaussian Kernel
	pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ>::Ptr kernel(new pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ>());
	kernel->setSigma(_sigma);
	kernel->setThresholdRelativeToSigma(3);

	//Set up the KDTree
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdTree(new pcl::search::KdTree<pcl::PointXYZ>);
	kdTree->setInputCloud(_cloud);

	//Set up the Convolution Filter
	pcl::filters::Convolution3D<pcl::PointXYZ, pcl::PointXYZ, pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ> > convolution;
	convolution.setKernel(*kernel);
	convolution.setInputCloud(_cloud);
	convolution.setSearchMethod(kdTree);
	convolution.setRadiusSearch(_radius);
	convolution.convolve(*smoothedCloud);

	return smoothedCloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Helper::MLSSmoothing(const pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud, const double _radius)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr smoothedCloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointNormal>::Ptr MLSPoints(new pcl::PointCloud<pcl::PointNormal>());

	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdTree(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
	mls.setComputeNormals(false);
	mls.setInputCloud(_cloud);
	mls.setPolynomialFit(true);
	mls.setSearchMethod(kdTree);
	mls.setSearchRadius(_radius);
	mls.process(*MLSPoints);

	pcl::copyPointCloud<pcl::PointNormal, pcl::PointXYZ>(*MLSPoints, *smoothedCloud);
	return smoothedCloud;
}

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr Helper::createColorCloud(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, uint32_t _color)
{
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr coloredCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
	coloredCloud->reserve(_cloud->size());

	float color = *reinterpret_cast<float*>(&_color);
	for (int i = 0; i < _cloud->width; i++)
	{
		pcl::PointNormal p = _cloud->points[i];
		coloredCloud->push_back(PointFactory::makePointXYZRGBNormal(p.x, p.y, p.z, p.normal_x, p.normal_y, p.normal_z, p.curvature, color));
	}

	return coloredCloud;
}

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr Helper::createColorCloud(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, uint8_t _r, uint8_t _g, uint8_t _b)
{
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr coloredCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
	coloredCloud->reserve(_cloud->size());

	for (int i = 0; i < _cloud->width; i++)
	{
		pcl::PointNormal p = _cloud->points[i];
		coloredCloud->push_back(PointFactory::makePointXYZRGBNormal(p.x, p.y, p.z, p.normal_x, p.normal_y, p.normal_z, p.curvature, _r, _g, _b));
	}

	return coloredCloud;
}

float Helper::getColor(const uint8_t _r, const uint8_t _g, const uint8_t _b)
{
	uint32_t color = ((uint32_t) _r << 16 | (uint32_t) _g << 8 | (uint32_t) _b);
	float finalColor = *reinterpret_cast<float*>(&color);
	return finalColor;
}

unsigned int Helper::getColor(const int _index)
{
	static uint32_t palette[12] =
	{ 0xa6cee3, 0x1f78b4, 0xb2df8a, 0x33a02c, 0xfb9a99, 0xe31a1c, 0xfdbf6f, 0xff7f00, 0xcab2d6, 0x6a3d9a, 0xffff99, 0xb15928 };

	return palette[_index % 12];
}

bool Helper::loadClusteringCache(cv::Mat &_descriptors, const ExecutionParams &_params)
{
	bool loadOK = true;
	std::string filename = _params.cacheLocation + _params.getHash();
	size_t row = 0;

	std::string line;
	std::ifstream cacheFile;
	cacheFile.open(filename.c_str(), std::fstream::in);
	if (cacheFile.is_open())
	{
		while (getline(cacheFile, line))
		{
			if (line.empty())
				continue;

			std::vector<std::string> tokens;
			std::istringstream iss(line);
			std::copy(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(), std::back_inserter(tokens));

			if (tokens.size() != _descriptors.cols)
			{
				loadOK = false;
				break;
			}

			for (size_t col = 0; col < tokens.size(); col++)
				_descriptors.at<float>(row, col) = (float) atof(tokens[col].c_str());
			row++;
		}
		cacheFile.close();
	}
	else
		loadOK = false;

	return loadOK;
}

void Helper::writeClusteringCache(const cv::Mat &_descriptors, const ExecutionParams &_params)
{
	std::string destination = _params.cacheLocation + _params.getHash();

	if (!boost::filesystem::exists(_params.cacheLocation))
		system(("mkdir " + _params.cacheLocation).c_str());

	std::ofstream cacheFile;
	cacheFile.open(destination.c_str(), std::fstream::out);

	for (int i = 0; i < _descriptors.rows; i++)
	{
		for (int j = 0; j < _descriptors.cols; j++)
			cacheFile << std::setprecision(15) << _descriptors.at<float>(i, j) << " ";
		cacheFile << "\n";
	}

	cacheFile.close();
}

double Helper::calculateSSE(const cv::Mat &_descriptors, const cv::Mat &_centers, const cv::Mat &_labels)
{
	double sse = 0;
	for (int i = 0; i < _descriptors.rows; i++)
	{
		float norm = cv::norm(_descriptors.row(i), _centers.row(_labels.at<int>(i)));
		sse += (norm * norm);
	}

	return sse;
}

void Helper::generateElbowGraph(const cv::Mat &_descriptors, const ExecutionParams &_params)
{
	int attempts = 5;
	cv::Mat labels, centers;

	std::ofstream data;
	data.open("./output/graph.dat", std::fstream::out);
	for (int i = 2; i < 50; i++)
	{
		cv::kmeans(_descriptors, i, labels, cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, _params.maxIterations, _params.stopThreshold), attempts, cv::KMEANS_PP_CENTERS, centers);
		double sse = Helper::calculateSSE(_descriptors, centers, labels);
		data << i << " " << sse << "\n";
	}
	data.close();

	std::ofstream graph;
	graph.open("./output/graph.plot", std::fstream::out);

	graph << "set xtic auto\n";
	graph << "set ytic auto\n";
	graph << "set grid ytics xtics\n\n";

	graph << "set title 'Sum of Squared Errors'\n";
	graph << "set xlabel 'Number of Clusters'\n";
	graph << "set ylabel 'SSE'\n\n";

	graph << "set term png\n";
	graph << "set output './output/elbowGraph.png'\n\n";

	graph << "plot './output/graph.dat' using 1:2 title 'SSE' with linespoints lt rgb 'blue'\n";

	graph.close();

	system("gnuplot ./output/graph.plot");
}
