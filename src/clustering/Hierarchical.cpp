/**
 * Author: rodrigo
 * 2016
 */
#include "Hierarchical.hpp"

void Hierarchical::agglomerativeClustering(ClusteringResults &_results, const cv::Mat &_items, const MetricPtr &_metric)
{
	// Generate a matrix between the elements
	cv::Mat distance = Clustering::generatePointDistanceMatrix(_items, _metric);

	// Set the initial mapping of indices
	std::map<int, int> indices;
	for (int i = 0; i < _items.rows; i++)
		indices[i] = i;

	// Iterate over all the points
	for (int i = 0; i < _items.rows; i++)
	{
		std::cout << i << std::endl;

		// Find closest clusters
		std::pair<int, int> closest = findClosetsClusters(distance);

		// Merge and delete closets clusters
//		mergeClusters(distance, closest.first, closest.second);



		// TODO add something to keep track of the merge of clusters, that is
		// which elements are being put in the same cluster, so later the cloud can be colored




		// Update distances
		int t = closest.first; // Put the merged cluster where the first cluster was
		for (int k = 0; k < distance.rows; k++)
		{
//			float newDistance = updatedDistance(distance, closest.first, closest.second); // TODO here the distance should be updated according to the selected linkage
//			distance.at<float>(k, t) = newDistance;
//			distance.at<float>(t, k) = newDistance;
		}






		// Delete individual clusters merged

		// Update distance matrix
	}
}

std::pair<int, int> Hierarchical::findClosetsClusters(const cv::Mat &_distanceMatrix)
{
	double minDist, maxDist;
	cv::Point min, max;
	cv::minMaxLoc(_distanceMatrix, &minDist, &maxDist, &min, &max);

	return std::pair<int, int>(min.x, min.y);
}

void Hierarchical::mergeClusters(const cv::Mat &_distanceMatrix, const int _cluster1, const int _cluster2)
{
}
