/**
 * Author: rodrigo
 * 2016
 */
#pragma once

#include "DHC.hpp"
#include "SHOT.hpp"


class DescriptorFactory
{
public:
	static DescriptorPtr create(const DescriptorType &type_)
	{
		DescriptorPtr descriptor;
		switch (type_)
		{
		case DESC_DCH:
			descriptor = DescriptorPtr(new DCH());
			descriptor->load(Config::getInstance()->get()["descriptor"]["DCH"]);
			return descriptor;

		case DESC_SHOT:
			descriptor = DescriptorPtr(new SHOT());
			descriptor->load(Config::getInstance()->get()["descriptor"]["SHOT"]);
			return descriptor;

		// case DESC_USC:
		// 	descriptor = DescriptorPtr(new USC());
		// descriptor->load(Config::getInstance()->get()["descriptor"]["USC"]);
		// return descriptor;

		// case DESC_PFH:
		// 	descriptor = DescriptorPtr(new PFH());
		// descriptor->load(Config::getInstance()->get()["descriptor"]["PFH"]);
		// return descriptor;

		// case DESC_ROPS:
		// 	descriptor = DescriptorPtr(new ROPS());
		// descriptor->load(Config::getInstance()->get()["descriptor"]["ROPS"]);
		// return descriptor;

		default:
			return descriptor;
		}
	};

private:
	DescriptorFactory();
	~DescriptorFactory();
};
