#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <nav_msgs/Odometry.h>

#include <iostream>
#include <fstream>
#include <string>

#include <iomanip>
#include <ctime>

// Create ofstream for file
std::ofstream gpsPointsFile;
std::ofstream fs;
// Create a name for file output
std::string filename = "/home/boost-3/Desktop/utmDataPoints.csv";


// Initialize flag for button state
bool button_press_state = false;

// Initialize for time
// std::time_t currTime;


void fix_cb (const nav_msgs::Odometry::ConstPtr& odomMsg)
{
	ROS_DEBUG_STREAM("Got into fix callback!");
	if (button_press_state)
	{
		ROS_INFO_STREAM("Saving current fix to: " << filename);

		// Save /fix message to a line in the file
			// Get current time
		std::time_t currTime = std::time(0);

			// Open csv file
		gpsPointsFile.open(filename.c_str(), std::ofstream::out | std::ofstream::app);

  			// write to file (Lat, lon, alt, status, service, time), no need for endl as asctime outputs it
		double temp_x = odomMsg->pose.pose.position.x;
		double temp_y = odomMsg->pose.pose.position.y;
		// ROS_INFO(fix_msg->latitude);

		gpsPointsFile << std::fixed << std::setprecision(8);

		gpsPointsFile << odomMsg->pose.pose.position.x << "," << odomMsg->pose.pose.position.y  << "," << odomMsg->pose.pose.position.z << "," << std::asctime(std::localtime(&currTime));

  		gpsPointsFile.close();

		// Wait a few seconds, software debounce for button press
		sleep(5);

		// Set button flag to false again;
		button_press_state = false;
	}
}


void joy_cb (const sensor_msgs::JoyConstPtr& joy_msg)
{
	ROS_DEBUG_STREAM("Got into joy callback!");

	if (!joy_msg->buttons[6] && joy_msg->buttons[7])
	{
		ROS_INFO_STREAM("Button pressed!");
		button_press_state = true;
	}
}



int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "utm_point_collector_node");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the joy stick
  ros::Subscriber sub1 = nh.subscribe ("/odometry/coarse_gps", 1, fix_cb);
  ros::Subscriber sub2 = nh.subscribe ("/joy_teleop/joy", 1, joy_cb);

  // Create file and write to headers
  gpsPointsFile.open(filename.c_str(), std::ofstream::out | std::ofstream::app);  // Open csv file
  	// write file headers
  gpsPointsFile << std::endl << "x coord.," << "y coord.," << "z coord.," << "Time" << std::endl;
  gpsPointsFile.close();

  ros::Rate r(10);
  while (ros::ok())
  {

    r.sleep();

    // Spin
    ros::spinOnce ();
  }





}