#include <stdio.h>
#include <stdlib.h>

#include "ros/ros.h"

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/Float64MultiArray.h"
#include <nav_msgs/Odometry.h>

// GPS waypoints, in order
float xPoints [] = {1048.54, 1043.08, 1037.05};
float yPoints [] = {897.03, 864.92, 831.92};

// Current array position, start at second waypoint
int i = 1;

ros::Publisher waypoint_pub;






void odomCallback(const nav_msgs::Odometry::ConstPtr& odomMsg)
{
	// Minimum distance before switching waypoints in meters
	double minDist = 0.5;

	std_msgs::Float64MultiArray array;

	// Get current position in global odom frame
	double currX = odomMsg->pose.pose.position.x;
	double currY = odomMsg->pose.pose.position.y;

	// Check current dist
	double dist = sqrt(pow(currX - xPoints[i], 2.0) + pow(currY - yPoints[i], 2.0));

	if (dist < minDist)
	{
		++i;  // increment counter
	}

	//Clear array
	array.data.clear();
	// Put in waypoints (y2 x2 y1 x1)
	array.data.push_back(yPoints[i]);
	array.data.push_back(xPoints[i]);
	array.data.push_back(yPoints[i-1]);
	array.data.push_back(xPoints[i-1]);

	// Publish waypoints
	ROS_INFO("Publishing waypoints!");
	waypoint_pub.publish(array);
}

int main(int argc, char **argv)
{
    
	ros::init(argc, argv, "arrayPublisher");

	ros::NodeHandle n;

	waypoint_pub = n.advertise<std_msgs::Float64MultiArray>("gps/waypoints", 10);

	ros::Subscriber odom_sub = n.subscribe("odometry/coarse_gps", 1, odomCallback);

	// while (ros::ok())
	// {
	// 	std_msgs::Float64MultiArray array;
	// 	//Clear array
	// 	array.data.clear();
	// 	// Put in waypoints (y2 x2 y1 x1)
	// 	array.data.push_back(897.03);
	// 	array.data.push_back(1048.54);
	// 	array.data.push_back(864.92);
	// 	array.data.push_back(1043.08);



	// 	//Publish array
	// 	pub.publish(array);
	// 	//Let the world know
	// 	ROS_INFO("Publishing waypoints!");
	// 	//Do this.
	// 	ros::spinOnce();
	// 	//Added a delay so not to spam
	// 	sleep(2);
	// }

	ros::Rate rate(10);
	while (ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
	}

}