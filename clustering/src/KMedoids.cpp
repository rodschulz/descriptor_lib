/**
 * Author: rodrigo
 * 2016     
 */
#include "KMedoids.hpp"
#include "ClusteringUtils.hpp"

void KMedoids::searchClusters(ClusteringResults &results_, const cv::Mat &items_, const MetricPtr &metric_, const int ncluster_, const int attempts_, const int maxIterations_, const double stopThreshold_)
{
	results_.prepare(ncluster_, items_.rows, items_.cols);

	cv::Mat distances = cv::Mat(items_.rows, items_.rows, CV_64FC1, -1);
	double minError = std::numeric_limits<double>::max();

	for (int i = 0; i < attempts_; i++)
	{
		std::cout << "\t** attempt " << i << std::endl;

		cv::Mat currentMedoids = cv::Mat::zeros(ncluster_, items_.cols, CV_32FC1);
		cv::Mat labels = cv::Mat::zeros(items_.rows, 1, CV_32SC1);
		std::vector<double> errorCurve;

		// Select some of the elements as the initial medoids
		std::vector<int> currentIndices = Utils::getRandomIntArray(ncluster_, 0, items_.rows - 1, false);

		// Set intial labeling
		for (int k = 0; k < items_.rows; k++)
			labels.at<int>(k) = findClosestCenter(items_, k, currentIndices, metric_, distances);

		// Calculate initial error
		extractMedoids(items_, currentIndices, currentMedoids);
		double currentError = ClusteringUtils::getSSE(items_, labels, currentMedoids, metric_);
		errorCurve.push_back(currentError);

		std::cout << "Sweeping points " << std::endl;

		// Sweep the space looking for new medoids
		int itCounter = 0;
		cv::Mat medoids = cv::Mat::zeros(ncluster_, items_.cols, CV_32FC1);
		for (size_t j = 0; j < currentIndices.size(); j++)
		{
			for (int k = 0; k < items_.rows; k++)
			{
				if (itCounter % 400 == 0)
					std::cout << "\tSweeped " << k << " points and " << j << " medoids" << std::endl;

				if (currentIndices[j] == k || std::find(currentIndices.begin(), currentIndices.end(), k) != currentIndices.end())
					continue;

				// Swap indices
				std::vector<int> indices = currentIndices;
				indices[j] = k;

				// Set labels
				for (int k = 0; k < items_.rows; k++)
					labels.at<int>(k) = findClosestCenter(items_, k, currentIndices, metric_, distances);

				// Calculate error
				extractMedoids(items_, indices, medoids);
				double error = ClusteringUtils::getSSE(items_, labels, medoids, metric_);

				if (error < currentError)
				{
					bool stop = ClusteringUtils::evaluateStopCondition(currentMedoids, medoids, stopThreshold_, metric_);

					// Update centers and error
					currentIndices = indices;
					currentError = error;
					medoids.copyTo(currentMedoids);

					// Store error evolution
					errorCurve.push_back(error);

					if (stop)
					{
						std::cout << "\tthreshold reached --> [attempt: " << i << " - iteration: " << itCounter << "]" << std::endl;
						break;
					}
				}

				itCounter++;
			}
		}

		std::cout << "\tSSE: " << std::fixed << errorCurve.back() << std::endl;
		if (errorCurve.back() < minError)
		{
			// Update the results
			labels.copyTo(results_.labels);
			extractMedoids(items_, currentIndices, currentMedoids);
			currentMedoids.copyTo(results_.centers);
			results_.errorEvolution = errorCurve;

			// Update the control variable
			minError = errorCurve.back();
		}
	}

	// Print a report of the results
	std::cout << "KMedoids finished -- Error: " << minError << "\n";
	std::vector<int> zeroPointClusters;

	std::vector<int> itemCount = ClusteringUtils::itemsPerCluster(ncluster_, results_.labels);
	for (size_t i = 0; i < itemCount.size(); i++)
	{
		if (itemCount[i] == 0)
			zeroPointClusters.push_back(i);
		std::cout << "\tcluster " << i << ": " << itemCount[i] << " points\n";
	}

	// Report zero point clusters
	std::cout << zeroPointClusters.size() << " clusters with no points" << std::endl;
	for (size_t i = 0; i < zeroPointClusters.size(); i++)
		std::cout << "\tcluster " << zeroPointClusters[i] << ": 0 points\n";
}

int KMedoids::findClosestCenter(const cv::Mat &items_, const int target_, const std::vector<int> &medoids_, const MetricPtr &metric_, cv::Mat &distances_)
{
	int closestMedoid = -1;

	// Find the closest centroid for the current descriptor
	double minDist = std::numeric_limits<double>::max();
	for (size_t i = 0; i < medoids_.size(); i++)
	{
		double distance = distances_.at<double>(i, target_);
		if (abs(distance + 1) < 1e-5)
		{
			distance = metric_->distance(items_.row(medoids_[i]), items_.row(target_));
			distances_.at<double>(i, target_) = distance;
			distances_.at<double>(target_, i) = distance;
		}

		if (distance < minDist)
		{
			minDist = distance;
			closestMedoid = i;
		}
	}

	return closestMedoid;
}

void KMedoids::extractMedoids(const cv::Mat &items_, const std::vector<int> &indices_, cv::Mat &medoids_)
{
	for (size_t i = 0; i < indices_.size(); i++)
		items_.row(indices_[i]).copyTo(medoids_.row(i));
}
