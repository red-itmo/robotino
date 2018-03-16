## Repository description
The repository contains a minimum set of ROS packages for working with [Robotino](http://www.festo-didactic.com/int-en/learning-systems/education-and-research-robots-robotino/the-complete-robotino-package.htm?fbid=aW50LmVuLjU1Ny4xNy4xOC44NTguNDc1Ng).
These packages are modified version of their namesakes from revision [3303](http://svn.openrobotino.org/!svn/bc/3303/robotino-ros-pkg/trunk/catkin-pkg/) of [the official repository](http://svn.openrobotino.org).

### The contents of the packages:
* robotino_node:

    ROS drivers for Robotino and some of extra devices such as webcam and lidar;

* robotino_msgs:

    files with description of msg and srv message types which are used within robotino_node package;

* robotino_description

    files with [URDF](http://wiki.ros.org/urdf) description of robot and some of extra devices such as webcam and lidar.


## Installation
1. Add a needed repository as it is described [here](http://wiki.openrobotino.org/index.php?title=Debrepository).
2. Update information about available repositories
    ```bash
    $ sudo apt-get update
    ```
3. Install RobotinoAPI2 library by typing the command
    ```bash
    $ sudo apt-get install robotino-api2
    ```
    **Note:** Alternatively RobotinoAPI2 library can be built from sources as it is described [here](http://wiki.openrobotino.org/index.php?title=API2_source_build).

4. Copy this repository into *src* subfolder of your workspace by for example this way
    ```bash
    $ cd PATH_TO_YOUR_WORKSPACE/src
    $ git clone https://github.com/red-itmo/robotino.git
    ```
5. Run catkin_make command:
    ```bash
    $ cd PATH_TO_YOUR_WORKSPACE
    $ catkin_make
    ```


## Decription of nodes from robotino_node package
Some part of the information below was taken from [robotino_node package description](http://wiki.ros.org/robotino_node) on appropriate page of [wiki.ros](http://wiki.ros.org).

### 1. robotino_node

#### Provided services
* set_gripper_state ([robotino_msgs/SetGripperState](https://github.com/red-itmo/robotino/blob/master/robotino_msgs/srv/SetGripperState.srv))

    Service to open or close the gripper

* set_encoder_position ([robotino_msgs/SetEncoderPosition](https://github.com/red-itmo/robotino/blob/master/robotino_msgs/srv/SetEncoderPosition.srv))

    Service to set the encoder position

* set_ns_ceil_height ([robotino_msgs/SetNsCeilHeight](https://github.com/red-itmo/robotino/blob/master/robotino_msgs/srv/SetNsCeilHeight.srv))

    Service to change [calibration parameter](http://doc.openrobotino.org/download/RobotinoAPI2/rec_robotino_api2/classrec_1_1robotino_1_1api2_1_1_north_star.html#2b02ea2d1ba9417c10276c0e36bc8747) for NorthStar

#### Subscribed topics
* set_digital_values ([robotino_msgs/DigitalReadings](https://github.com/red-itmo/robotino/blob/master/robotino_msgs/msg/DigitalReadings.msg))

    Digital values for digital outputs on Robotino are set using this topic

* cmd_vel ([geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html))

    Allows setting velocities for Robotino

####  Published topics
* analog_readings ([robotino_msgs/AnalogReadings](https://github.com/red-itmo/robotino/blob/master/robotino_msgs/msg/AnalogReadings.msg))

    Readings from the analog inputs on Robotino

* bumper ([std_msgs/Bool](http://docs.ros.org/api/std_msgs/html/msg/Bool.html))

    Readings from the bumper sensor on Robotino

* digital_readings ([robotino_msgs/DigitalReadings](https://github.com/red-itmo/robotino/blob/master/robotino_msgs/msg/DigitalReadings.msg))

    Readings from the digital inputs on Robotino

* distance_sensors ([sensor_msgs/PointCloud](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud.html))

    Readings from the distance sensors on Robotino

* gripper_state ([robotino_msgs/GripperState](https://github.com/red-itmo/robotino/blob/master/robotino_msgs/msg/GripperState.msg))

    Readings from the gripper on Robotino (it currently doesn't work properly)

* encoder_readings ([robotino_msgs/EncoderReadings](https://github.com/red-itmo/robotino/blob/master/robotino_msgs/msg/EncoderReadings.msg))

    Readings from additional Robotino's encoder input

* motor_readings ([robotino_msgs/MotorReadings](https://github.com/red-itmo/robotino/blob/master/robotino_msgs/msg/MotorReadings.msg))

    Readings from Robotino's motors

* north_star ([robotino_msgs/NorthStarReadings](https://github.com/red-itmo/robotino/blob/master/robotino_msgs/msg/NorthStarReadings.msg))

    Readings from the NorthStar sensor on Robotino

* power_readings ([robotino_msgs/PowerReadings](https://github.com/red-itmo/robotino/blob/master/robotino_msgs/msg/PowerReadings.msg))

    Power readings from Robotino (it doesn't work)

* robotino_joint_states ([sensor_msgs/JointState](http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html))

    Information about positions of robot's joints

#### Parameters
* ~hostname (string, default: 172.26.1.1)

    Robotino's IP Address

* ~max_linear_vel (double, default: 0.2)

    Maximum linear velocity in m/s

* ~min_linear_vel (double, default: 0.05)

    Minimum linear velocity in m/s

* ~max_angular_vel (double, default: 1.0)

    Maximum angular velocity in rad/s

* ~min_angular_vel (double, default: 0.1)

    Minimum angular velocity in rad/s

* ~ns_room_id (int, default: 3)

	The [room id](http://doc.openrobotino.org/download/RobotinoAPI2/rec_robotino_api2/classrec_1_1robotino_1_1api2_1_1_north_star.html#d85bee0ef84c90904c5a04d2357424c5) parameter for NorthStar

* ~ns_ceil_height (double, default: 3.0)

    [Calibration parameter](http://doc.openrobotino.org/download/RobotinoAPI2/rec_robotino_api2/classrec_1_1robotino_1_1api2_1_1_north_star.html#2b02ea2d1ba9417c10276c0e36bc8747) for NorthStar

* ~tf_prefix(string, default: "no_prefix")

    If value of this parameter isn't equal to "no_prefix", all robot's links are put to subspace which name coincides with the value of this parameter; for example, when tf_prefix = "robot1", link "base_link" gets new name "robot1/base_link"

### 2. robotino_odometry_node

#### Provided services
* reset_odometry ([robotino_msgs/ResetOdometry](https://github.com/red-itmo/robotino/blob/master/robotino_msgs/srv/ResetOdometry.srv))

    Service to reset the odometry

#### Published topics
* odom ([nav_msgs/Odometry](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html))

    Odometry data from robot

#### Parameters
* ~hostname (string, default: 172.26.1.1)

    Robotino's IP Address

* ~tf_prefix(string, default: "no_prefix")

    If value of this parameter isn't equal to "no_prefix", "odom" and "base_link" links are put to subspace which name coincides with the value of this parameter; for example, when tf_prefix = "robot1", link "base_link" gets new name "robot1/base_link"

#### Provided tf transforms
* odom --> base_link

    The node updates this transform using odometry data

### 3. robotino_laserrangefinder_node

#### Published topics
* scan ([sensor_msgs/LaserScan](http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html))

    Provides data from lidar

#### Parameters
* ~hostname (string, default: 172.26.1.1)

    Robotino's IP Address

* ~tf_prefix(string, default: "no_prefix")

    If value of this parameter isn't equal to "no_prefix", "laser_link" link is put to subspace which name is coincide with the value of this parameter; for example, when tf_prefix = "robot1", link "laser_link" gets new name "robot1/laser_link"

* ~laserRangeFinderNumber(int, default: 0)

    The number of lidar. If it is equal to 0, then lidar has topic "scan" and link "laser_link". If it is equal to 1, then lidar has topic "scan1" and link "laser_link1". And so on.

### 4. robotino_camera_node

#### Published topics
* image_raw ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))

    Provides image from webcam

#### Parameters
* ~hostname (string, default: 172.26.1.1)

    Robotino's IP Address

* ~cameraNumber(int, default: 0)

    The number of webcam. If it is equal to 0, then image from webcam is available from topic "image_raw". If it is equal to 1, then it uses topic "image_raw1". And so on.


## Launching
All nodes which are needed for work of only one robot can be started using this command
```bash
$ roslaunch robotino_node robotino_node.launch hostname:=IP_ADRESS_OF_YOUR_ROBOT
```
For several robots (for 2 in example below) type these commands in different terminals:
```bash
$ roslaunch robotino_node robotino_node.launch hostname:=IP_ADRESS_OF_FIRST_ROBOT tf_prefix:=robot1 use_tf_prefix:=true __ns:=robot1
```
```bash
$ roslaunch robotino_node robotino_node.launch hostname:=IP_ADRESS_OF_SECOND_ROBOT tf_prefix:=robot2 use_tf_prefix:=true __ns:=robot2
```
