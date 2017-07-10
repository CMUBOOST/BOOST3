#include <stdio.h>
#include <stdlib.h>

#include "ros/ros.h"

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/Float64MultiArray.h"

int main(int argc, char **argv)
{
    

	ros::init(argc, argv, "arrayPublisher");

	ros::NodeHandle n;

	ros::Publisher pub = n.advertise<std_msgs::Float64MultiArray>("gps/waypoints", 100);

	while (ros::ok())
	{
		std_msgs::Float64MultiArray array;
		//Clear array
		array.data.clear();
		// Put in waypoints (y2 x2 y1 x1)
		array.data.push_back(897.03);
		array.data.push_back(1048.54);
		array.data.push_back(864.92);
		array.data.push_back(1043.08);



		//Publish array
		pub.publish(array);
		//Let the world know
		ROS_INFO("Publishing waypoints!");
		//Do this.
		ros::spinOnce();
		//Added a delay so not to spam
		sleep(2);
	}

}