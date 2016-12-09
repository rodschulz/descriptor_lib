/**
 * Author: rodrigo
 * 2016
 */
#pragma once

#include <string>
#include <boost/shared_ptr.hpp>


/**************************************************/
struct DescriptorParamsXX
{
	DescriptorParamsXX() {};
	virtual ~DescriptorParamsXX() {};
	// virtual std::string toString() const = 0;
};
typedef boost::shared_ptr<DescriptorParamsXX> DescriptorParamsPtr;


/**************************************************/
struct DCHParams: public DescriptorParamsXX
{
	DCHParams()
	{
		searchRadius = 0;
	};
	~DCHParams() {};

	float searchRadius;
};


/**************************************************/
struct SHOTParams: public DescriptorParamsXX
{
	SHOTParams()
	{
		searchRadius = 0;
	};
	~SHOTParams() {};

	float searchRadius;
};


// /**************************************************/
// struct USCParams: public DescriptorParams
// {};


// /**************************************************/
// struct PFHParams: public DescriptorParams
// {};
