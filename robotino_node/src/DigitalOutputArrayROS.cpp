/*
 * DigitalOutputArrayROS.cpp
 *
 *  Created on: 09.12.2011
 *      Author: indorewala@servicerobotics.eu
 */

#include "DigitalOutputArrayROS.h"

DigitalOutputArrayROS::DigitalOutputArrayROS()
{
	digital_sub_ = nh_.subscribe("set_digital_values", 1,
			&DigitalOutputArrayROS::setDigitalValuesCallback, this);
}

DigitalOutputArrayROS::~DigitalOutputArrayROS()
{
	digital_sub_.shutdown();
}

void DigitalOutputArrayROS::setDigitalValuesCallback( const robotino_msgs::DigitalReadingsConstPtr& msg)
{
	if(msg->values.size() == numDigitalOutputs()){

		int values[numDigitalOutputs()];
		unsigned int i;

		for(i = 0; i < numDigitalOutputs(); i++)
			values[i] = msg->values[i];

		setValues(values, numDigitalOutputs());
	}
}
