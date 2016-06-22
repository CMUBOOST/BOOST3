#include "ros/ros.h"
#include <sstream>
#include <string>
#include <vector>
#include <math.h>

#include "sensor_msgs/Imu.h"

std::vector<double> orientation (4, 0);
std::vector<double> angular_velocity (3, 0);
std::vector<double> linear_acceleration (3, 0);

/**
 *  Callback for grabbing IMU data from UM7
*/
void IMUCovarianceCallback(const sensor_msgs::Imu::ConstPtr& msg) {
	
	
}










int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "stalker_interface");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  ros::Subscriber sub_joint_command = n.subscribe("um7/imu/unfiltered_data", 10, IMUCovarianceCallback);



}