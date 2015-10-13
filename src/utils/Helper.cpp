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
#include <Eigen/Geometry>
#include <ctype.h>
#include "ExecutionParams.h"
#include "../factories/CloudFactory.h"
#include "../factories/PointFactory.h"

boost::random::mt19937 randomGenerator;

Helper::Helper()
{
}

Helper::~Helper()
{
}

int Helper::getRandomNumber(const int _min, const int _max)
{
	randomGenerator.seed(std::time(0));
	boost::random::uniform_int_distribution<> dist(_min, _max);
	return dist(randomGenerator);
}

std::vector<int> Helper::getRandomSet(const unsigned int _size, const int _min, const int _max)
{
	randomGenerator.seed(std::time(0));
	boost::random::uniform_int_distribution<> dist(_min, _max);

	std::vector<int> numbers;
	numbers.reserve(_size);

	std::map<int, bool> used;
	while (numbers.size() < _size)
	{
		int number = dist(randomGenerator);
		if (used.find(number) == used.end())
			numbers.push_back(number);
	}

	return numbers;
}

void Helper::removeNANs(pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud)
{
	std::vector<int> mapping;
	pcl::removeNaNFromPointCloud(*_cloud, *_cloud, mapping);
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

			default:
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

			loadOK = (int) tokens.size() == _descriptors.cols;
			if (!loadOK)
				break;

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
		if (system(("mkdir " + _params.cacheLocation).c_str()) != 0)
			std::cout << "WARNING: can't create clustering cache folder" << std::endl;

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

	if (system("gnuplot ./output/graph.plot") != 0)
		std::cout << "WARNING: can't execute GNUPlot" << std::endl;
}

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr Helper::generateClusterRepresentation(const pcl::PointCloud<pcl::PointNormal>::Ptr _cloud, const cv::Mat &_labels, const cv::Mat &_centers, const ExecutionParams &_params)
{
	int sequenceLength = _params.getSequenceLength();
	std::vector<pcl::PointNormal> locations(_centers.rows, PointFactory::makePointNormal(0, 0, 0, 0, 0, 0, 0));
	std::vector<int> pointsPerCluster(_centers.rows, 0);

	for (size_t i = 0; i < locations.size(); i++)
	{
		locations[i].x = _params.patchSize;
		locations[i].z = _params.patchSize * 3 * i;
	}

	// Angular step between bands
	double bandAngularStep = _params.getBandsAngularStep();

	// Define reference vectors
	Eigen::Vector3f referenceNormal = Eigen::Vector3f(1, 0, 0);
	Eigen::Vector3f referenceRotationAxis = Eigen::Vector3f(0, 1, 0);

	// Create the vectors placed according to the cluster centroid info
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
	for (int i = 0; i < _centers.rows; i++)
	{
		for (int j = 0; j < _params.bandNumber; j++)
		{
			Eigen::Vector3f baseLocation = locations[i].getVector3fMap();

			// Calculate the normal's rotation axis according to the current band's angle
			double bandAngle = bandAngularStep * j;
			Eigen::Vector3f rotationAxis = Eigen::AngleAxis<float>(bandAngle, referenceNormal).matrix() * referenceRotationAxis;
			rotationAxis.normalize();

			for (int k = 0; k < sequenceLength; k++)
			{
				// Calculate the rotated normal according to the angle of the current bin in the current band
				float angle = _centers.at<float>(i, j * sequenceLength + k);
				Eigen::Vector3f rotatedNormal = Eigen::AngleAxis<float>(angle, rotationAxis).matrix() * referenceNormal;
				rotatedNormal.normalize();

				// Create a plane oriented according to the rotated normal vector
				Eigen::Hyperplane<float, 3> plane;
				if (output->empty())
					plane = Eigen::Hyperplane<float, 3>(rotatedNormal, baseLocation);
				else
					plane = Eigen::Hyperplane<float, 3>(rotatedNormal, output->back().getVector3fMap());

				// Add 9 points to represent the bin
				for (double sideStep = -_params.bandWidth / 2; sideStep < _params.bandWidth; sideStep += _params.bandWidth / 2)
				{
					for (double forwardStep = 0; forwardStep < _params.sequenceBin; forwardStep += _params.sequenceBin / 3)
					{
						float x = 0;
						float y = sideStep;
						float z = forwardStep + k * _params.sequenceBin;
						Eigen::Vector3f displaced = Eigen::Vector3f(x, y, z);
						Eigen::Vector3f point = Eigen::AngleAxis<float>(bandAngle, referenceNormal) * displaced;
						point = point + Eigen::Vector3f(baseLocation.x(), baseLocation.y(), baseLocation.z());

						// Project the point over a plane oriented according the normal of the band's bin
						//Eigen::Vector3f p = plane.projection(point);
						Eigen::Vector3f p = point;

						float nx = rotatedNormal.x();
						float ny = rotatedNormal.y();
						float nz = rotatedNormal.z();

						output->push_back(PointFactory::makePointXYZRGBNormal(p.x(), p.y(), p.z(), nx, ny, nz, 0, Helper::getColor(i)));
					}
				}
			}
		}
	}

	return output;
}
