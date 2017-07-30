#include <stdio.h>
#include <stdlib.h>

#include "ros/ros.h"

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/Float64MultiArray.h"
#include <nav_msgs/Odometry.h>

// GPS waypoints, in order
// float xPoints [] = {};
// float yPoints [] = {};

float xPoints [] = {1095.82,1093.83,1112.76,1141.48,1164.64,1199.11,1211.82,1214.94,1212.27,1192.18,1178.92,1157.01,1127.27,1114.62,1093.53,1072.35,1050.89,1001.18,933.16,866.19,850.8,772.05,761.09,755.83,749.21,741.11,740.81,756.94,770.25,780.71,790.21,796.73,805.83,810.33,817.41,822.59,826.14,830.46,827.77,822.34,795.54,777.46,768.61,757.37,727.92,700.02,674.83,637.58,626.65,614.01,605.17,598.49,591.36,575.68,567.59,552.25,539,521.12,507.94,495.36,476.96,431.47,409.66,386.94,367.92,351.91,343.36,334.03,323.15,314.81,308.9,309.67};
float yPoints [] = {949.29,920.75,903.35,886.06,870.68,837.09,819.06,811.72,802.16,779.17,769.72,764.55,753.69,753.12,756.79,758.66,763.64,775.92,793.78,822.27,830.41,873.53,879.26,881.64,879.76,873.4,868.87,845.38,827,810.63,792.77,775.55,749.69,733.57,704.3,676.85,658.21,635.57,630.5,632.54,643.5,643.47,640.67,634.15,612.48,595.96,583.62,566.73,562.2,557.28,556.38,558.52,566.2,587.41,598.07,618.32,636.03,659.71,677.65,694.75,716.64,750.66,762.79,776.68,791.36,808.62,817.84,820.95,822.55,824.87,829.72,836.79};

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
		ROS_INFO_STREAM("Got to waypoint " << i);
	}

	//Clear array
	array.data.clear();
	// Put in waypoints (x1 y1 x2 y2)
	array.data.push_back(xPoints[i-1]);
	array.data.push_back(yPoints[i-1]);
	array.data.push_back(xPoints[i]);
	array.data.push_back(yPoints[i]);

	// Publish waypoints
	// ROS_INFO("Publishing waypoints!");
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