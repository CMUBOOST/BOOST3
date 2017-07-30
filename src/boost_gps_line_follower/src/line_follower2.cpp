#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>

#include "tf/transform_datatypes.h"

#include <sstream>
#include <string>
#include <vector>
#include <math.h> 

const double pi = 4*atan(1);
double currTime = 0.0;
double prevTime = 0.0;
// GPS Waypoint Data
double p0_x, p0_y, p1_x, p1_y;
bool waypointFlag = false;
// heading vector between the two input waypoints
double n[2];

double lookahead_distance;
double forward_velocity;

double omega; // angular velocity
int loop_rate = 5; // in Hz
double omegaGain = 20;  //5
double max_twist = 8; // (4) max angular velocity in rad per second


void waypointCallback(const std_msgs::Float64MultiArray::ConstPtr& waypoint_msg)
{		

	ROS_DEBUG_STREAM("Got into Waypoint Callback!");
	
	p0_x = waypoint_msg->data[0];
	p0_y = waypoint_msg->data[1];
	p1_x = waypoint_msg->data[2];
	p1_y = waypoint_msg->data[3];

	// Calculate heading between two way points
	double norm_n = sqrt(pow(p1_x - p0_x, 2) + pow(p1_y - p0_y, 2));
	n[0] = (p1_x - p0_x) / norm_n;
	n[1] = (p1_y - p0_y) / norm_n;
	ROS_DEBUG_STREAM("Heading is: " << n[0] << ", " << n[1]);
	
	ROS_DEBUG_STREAM("Setting waypoint flag to true!");
	waypointFlag = true;

	return;	
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
	
	if (waypointFlag)
	{
		ROS_DEBUG_STREAM("Got into Odom Callback!");
		
		// Get Yaw from quaternion orientation within coarse odometry message
		tf::Quaternion q(odom_msg->pose.pose.orientation.x, 
				 odom_msg->pose.pose.orientation.y, 
				 odom_msg->pose.pose.orientation.z, 
				 odom_msg->pose.pose.orientation.w);
		tf::Matrix3x3 m(q);
		double roll, pitch, yaw; 
		m.getRPY(roll, pitch, yaw); // Yaw should equal the heading
		
		double heading = yaw;
		// Get robot pose data
		double r_x = odom_msg->pose.pose.position.x;
		double r_y = odom_msg->pose.pose.position.y;
		
		printf("r_x = %0.6f\tr_y = %0.6f\n", r_x, r_y);		
		// error vector where e[0] = x comp and e[1] = y comp	
		double e[2]; 
		
		// Calculate perpendicular error vector between robot position and line between waypoints
		double eqn1 = pow(p0_x, 2) - 2 * p0_x * p1_x + pow(p1_x, 2) + pow(p0_y, 2) - 2 * p0_y * p1_y + pow(p1_y, 2);
		double eqn2 = p0_y * (r_x - p1_x) + p1_y * (p0_x - r_x) + r_y * (p1_x - p0_x);

		e[0] = - (p0_y - p1_y) * eqn2 / eqn1;
		e[1] = (p0_x - p1_x) * eqn2 / eqn1;
		double e_mag = pow(e[0]*e[0] + e[1]*e[1], 0.5);
		
		double theta; // angle between heading and vector from current position to lookahead point
		/* 
		double beta = heading; // adjusted heading angle
		double phi = atan(lookahead_distance/e_mag);
		
		double n_cross_e = n[0]*e[1] - n[1]*e[1];
		if (n_cross_e == 0) theta = 0;
		else if (n_cross_e > 0) {
			beta = pi - heading;
			theta = beta - phi;
		}
		else if (n_cross_e < 0) {
			theta = phi - beta;
		}
		*/
		double Lleg = pow(pow((p1_x-p0_x),2) + pow((p1_y-p0_y),2), 0.5);
		double Lproj = ((p1_x-p0_x)*(r_x-p0_x)+(p1_y-p0_y)*(r_y-p0_y))/Lleg;
		double Llookpoint = Lproj + lookahead_distance;
		double x_look = Llookpoint*((p1_x-p0_x)/Lleg) + p0_x;
		double y_look = Llookpoint*((p1_y-p0_y)/Lleg) + p0_y;
		double l[2];
		// Vector from robot position to lookahead point
		l[0] = x_look - r_x;
		l[1] = y_look - r_y;
		double l_mag = pow(l[0]*l[0] + l[1]*l[1], 0.5);
		double h[2];
		h[0] = cos(heading);
		h[1] = sin(heading);
		double alpha = acos((h[0]*l[0] + h[1]*l[1])/(l_mag*1));
		double h_cross_l = h[0]*l[1] - h[1]*l[0];
		if (h_cross_l == 0) theta = 0;
		else if (h_cross_l > 0) theta = alpha;
		else if (h_cross_l < 0) theta = -alpha;
		
		omega = theta*omegaGain;
		if ((fabs(omega) > max_twist) && (omega < 0)) omega = -max_twist;
		else if ((fabs(omega) > max_twist) && (omega > 0)) omega = max_twist;

		printf("n_x = %0.6f\tn_y = %0.6f\n", n[0], n[1]);
		printf("e_x = %0.6f\te_y = %0.6f\n", e[0], e[1]);
		printf("l_x = %0.6f\tl_y = %0.6f\n", l[0], l[1]);
		ROS_DEBUG_STREAM("h x l =   " << h_cross_l);
		ROS_INFO_STREAM("Heading = " << heading << ",\t" << heading*180/pi);
		ROS_INFO_STREAM("Desired heading = " << atan2(n[1], n[0]));
		ROS_DEBUG_STREAM("Lookvect = " << atan2(l[1],l[0]) << ",\t" << atan2(l[1],l[0])*180/pi);
		ROS_DEBUG_STREAM("N vector = " << atan2(n[1], n[0]) << ",\t" << atan2(n[1],n[0])*180/pi);
		ROS_DEBUG_STREAM("E mag =   " << e_mag);
		ROS_DEBUG_STREAM("Alpha =   " << alpha);
		// ROS_INFO_STREAM("Beta =    " << beta);
		// ROS_INFO_STREAM("Phi =     " << phi);
		ROS_DEBUG_STREAM("Theta =   " << theta << ",\t" << theta*180/pi);
		ROS_DEBUG_STREAM("Omega =   " << omega << "\n");
		
		prevTime = currTime;
		currTime = odom_msg->header.stamp.sec + (1e-9 * odom_msg->header.stamp.nsec);

		return;	
	}
	else 
	{
		ROS_DEBUG_STREAM("Haven't gotten a waypoint yet!");
		return;
	}
}


int main(int argc, char **argv)
{

	const double MAX_TIME = 5; // seconds
	
	ros::init(argc, argv, "boost_line_follower2");

	// Use message_filters to combine and sync two topics
	ros::NodeHandle nh;
	ros::Subscriber odom_sub = nh.subscribe("odometry/coarse_gps", 1, odomCallback);
	// ros::Subscriber odom_sub = nh.subscribe("odometry/filtered_imu_encoders", 1, odomCallback);
	ros::Subscriber waypoint_sub = nh.subscribe("gps/waypoints", 1, waypointCallback);

	ros::NodeHandle priv_node("~");
	priv_node.param<double>("lookahead_distance", lookahead_distance, 3);
	priv_node.param<double>("forward_velocity", forward_velocity, 0.3);

	// Initialize twist publisher, which will hold the velcity command information calculated in the callBack
	ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("nav_vel", 1);

	currTime = ros::Time::now().toSec();

	ros::spinOnce();  


	ros::Rate r(loop_rate);
	while (ros::ok())
	{

		
		double dt = currTime - prevTime;

		if (!waypointFlag)
		{
			ROS_DEBUG_STREAM("Waypoint not set yet!");

			r.sleep();
			ros::spinOnce();
			continue;
		}
		else if (dt > MAX_TIME)
		{
			ROS_DEBUG_STREAM("Time between cycles is " << dt << ", which is greater than " << MAX_TIME << "!");

			r.sleep();
			ros::spinOnce();
			continue;
		}
	
		// Populate Twist message
		geometry_msgs::Twist twist_msg;
		twist_msg.linear.x =  forward_velocity; // Fill with constant linear velocity
		twist_msg.angular.z = omega; // Fill with calculated angular velocity		
		
		// Publish Twist message
		twist_pub.publish(twist_msg);			

		r.sleep();
		ros::spinOnce();

	}
	return 0;

}
