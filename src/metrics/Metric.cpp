/**
 * Author: rodrigo
 * 2016
 */
#include "Metric.hpp"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include "MetricFactory.hpp"

Metric::Metric()
{
}

Metric::~Metric()
{
}

void Metric::evaluateMetricCases(const std::string &_resultsFilename, const std::string &_casesFilename, const MetricType &_metricType, const std::vector<std::string> &_args)
{
	std::cout << "Evaluating metric testcases" << std::endl;
	MetricPtr targetMetric = MetricFactory::createMetric(_metricType, _args);

	// Extract the testcases
	boost::property_tree::ptree tree;
	boost::property_tree::read_json(_casesFilename, tree);

	std::cout << "Test cases loaded" << std::endl;

	int k = 0;
	bool shift = true;
	std::vector<std::vector<float> > data1, data2;
	BOOST_FOREACH(boost::property_tree::ptree::value_type & testCase, tree.get_child("vectors"))
	{
		// Generate arrays for the vectors of this testcase
		data1.push_back(std::vector<float>());
		data2.push_back(std::vector<float>());

		// Iterate over the array of vectors
		BOOST_FOREACH(boost::property_tree::ptree::value_type & vectors, testCase.second)
		{
			// Extract data for a vector
			BOOST_FOREACH(boost::property_tree::ptree::value_type & vector, vectors.second)
			{
				float value = vector.second.get_value<float>();
				shift ? data1[k].push_back(value) : data2[k].push_back(value);
			}

			shift = !shift;
		}
		k++;
	}

	std::cout << "Evaluating metric" << std::endl;

	// Evaluate each testcase and write the results
	std::ofstream results;
	results.open(_resultsFilename.c_str(), std::ofstream::out);
	for (size_t i = 0; i < data1.size(); i++)
	{
		cv::Mat vector1 = cv::Mat(data1[i]).t();
		cv::Mat vector2 = cv::Mat(data2[i]).t();
		double distance = targetMetric->distance(vector1, vector2);

		results << vector1 << "\n" << vector2 << "\ndistance = " << distance << "\n\n";
	}
	results.close();

	std::cout << "Metric evaluated" << std::endl;
}
