/*
 * NorthStarROS.cpp
 *
 *  Created on: 08.12.2011
 *      Author: indorewala@servicerobotics.eu
 */

#include "NorthStarROS.h"

#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>

NorthStarROS::NorthStarROS()
{
	north_star_pub_ = nh_.advertise<robotino_msgs::NorthStarReadings>("north_star", 1, true);
	set_ns_ceil_height_server_ = nh_.advertiseService("set_ns_ceil_height", &NorthStarROS::setNsCeilHeightCallback, this);
}

NorthStarROS::~NorthStarROS()
{
	north_star_pub_.shutdown();
	set_ns_ceil_height_server_.shutdown();
}

void NorthStarROS::setTimeStamp(ros::Time stamp)
{
	stamp_ = stamp;
}

bool NorthStarROS::setNsCeilHeightCallback(robotino_msgs::SetNsCeilHeight::Request &req, robotino_msgs::SetNsCeilHeight::Response &res)
{
	if(req.height > 0.0){
		setCeilingCal(req.height);
		res.success = true;
		res.message = "Height's value was successfullly changed";
	}
	else{
		res.success = false;
		res.message = "Height's value must be greater than zero";
	}
	return true;
}

void NorthStarROS::readingsEvent( const rec::robotino::api2::NorthStarReadings& readings )
{
	geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(readings.posTheta);
	// Build the NorthStarReadings msg
	north_star_msg_.stamp 				= stamp_;
	north_star_msg_.seq 				= readings.sequenceNumber;
	north_star_msg_.roomId 				= readings.roomId;
	north_star_msg_.numSpotsVisible 	= readings.numSpotsVisible;
	north_star_msg_.pose.position.x 	= -readings.posY;
	north_star_msg_.pose.position.y 	= readings.posX;
	north_star_msg_.pose.position.z 	= 0.0;
	north_star_msg_.pose.orientation	= quat;

	// Publish the message
	north_star_pub_.publish( north_star_msg_ );
}
