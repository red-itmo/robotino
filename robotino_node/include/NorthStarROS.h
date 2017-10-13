/*
 * NorthStarROS.h
 *
 *  Created on: 08.12.2011
 *      Author: indorewala@servicerobotics.eu
 */

#ifndef NORTHSTARROS_H_
#define NORTHSTARROS_H_

#include "rec/robotino/api2/NorthStar.h"

#include <ros/ros.h>
#include "robotino_msgs/NorthStarReadings.h"
#include "robotino_msgs/SetNsCeilHeight.h"

class NorthStarROS : public rec::robotino::api2::NorthStar
{
public:
	NorthStarROS();
	~NorthStarROS();

	void setTimeStamp(ros::Time stamp);

private:
	ros::NodeHandle nh_;

	ros::Publisher north_star_pub_;

	robotino_msgs::NorthStarReadings north_star_msg_;

	ros::ServiceServer set_ns_ceil_height_server_;

	ros::Time stamp_;

	bool setNsCeilHeightCallback(robotino_msgs::SetNsCeilHeight::Request &req, robotino_msgs::SetNsCeilHeight::Response &res);

	void readingsEvent( const rec::robotino::api2::NorthStarReadings& readings );
};

#endif /* NORTHSTARROS_H_ */
