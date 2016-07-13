#include "ros/ros.h"
#include "std_msgs/Float32.h"

#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>

#include "tf/transform_broadcaster.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sstream>
#include <string>
#include <vector>
#include <math.h>

const double MIN_SPEED = 0.1; // m/s
const double toRadians = 3.14159265359 / 180.0;

double prevLat = 0.0;
double prevLon = 0.0;
double currLat = 0.0;
double currLon = 0.0;
double prevTime = 0.0;
double currTime = 0.0;
double prevSpeed = 0.0;
double currSpeed = 0.0;

ros::Time curr_time;

// TODO: ADD ALTITUDE TO GET 3D POSE ESTIMATE

void gpsHeadingCallback(const sensor_msgs::NavSatFix::ConstPtr& fix_msg, const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& twist_msg)
{
	ROS_DEBUG_STREAM("Got into callback!");

	// Add latitude and longitude to respective arrays, along with time, check for GBAS and fix
	if ((fix_msg->status.status > -1) && (fabs(twist_msg->twist.twist.linear.x) > MIN_SPEED))
	{
		prevLat = currLat;
		prevLon = currLon;
		prevTime = currTime;
		prevSpeed = currSpeed;

		currLat = fix_msg->latitude * toRadians;
		currLon = fix_msg->longitude * toRadians;
		currTime = fix_msg->header.stamp.sec + (1e-9 * fix_msg->header.stamp.nsec);

		currSpeed = twist_msg->twist.twist.linear.x;

		curr_time = ros::Time::now();
		return;
	} 
	else 
	{
		ROS_DEBUG_STREAM("GBAS is not available or moving too slow!  GPS Status: " << fix_msg->status.status << "   Linear Speed: " << twist_msg->twist.twist.linear.x);
		return;
	}
	// TODO: Perform line fit on n previous fixes, get heading that way
}


int main(int argc, char **argv)
{

	const double MIN_DIST = 0.1; // meters, needs to be at least 3 times the standard deviation of the GPS signal
	const double MAX_TIME = 3; // seconds
	const double pi = 3.14159265359;
	const double radiusEarth = 6372795.477598;  // Radius of earth in meters

	ros::init(argc, argv, "boost_heading_from_gps_publisher");

	// Use message_filters to combine and sync two topics
	ros::NodeHandle nh;
	message_filters::Subscriber<sensor_msgs::NavSatFix> fix_sub(nh, "fix", 1);
	message_filters::Subscriber<geometry_msgs::TwistWithCovarianceStamped> twist_sub(nh, "base/cmd_vel", 1);

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::NavSatFix, geometry_msgs::TwistWithCovarianceStamped> MySyncPolicy;

	// ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), fix_sub, twist_sub);
	sync.registerCallback(boost::bind(&gpsHeadingCallback, _1, _2));

	// Initialize Imu publisher, which will hold the heading information calculated in the callBack
	// Imu is being used to spoof the navsat transform in robot_localization, kind of a hack...
	ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("gps/imu/data", 1, false);

	// Initialize pose publisher, which will hold the heading information calculated in the callBack
	// ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("gpsHeading", 1);

	ros::Time last_time;
	last_time = ros::Time::now();

	// ros::Subscriber sub_fix = nh.subscribe("fix", 1, gpsHeadingCallback);

	// Sleep to allow time for fix to be updated, HACK- FIX LATER
	sleep(1);
	ros::spinOnce();
	sleep(0.5);
	ros::spinOnce();  // Fill in currLat

	double latCheck = currLat;
	double lonCheck = currLon;

	ros::Rate r(0.5);
	while (ros::ok())
	{
		// Check that lat and long were updated
		if (latCheck == currLat && lonCheck == currLon)
		{
			ROS_DEBUG_STREAM(" Lat/Lons have not been updated!");
			r.sleep();
			ros::spinOnce();
			continue;
		}
		double latCheck = currLat;
		double lonCheck = currLon;

		// Latitudes and longitudes are in radians at this point!
		double dt = (curr_time - last_time).toSec();

		if (dt > MAX_TIME)
		{
			ROS_DEBUG_STREAM("Time between coordinates is " << dt << ", which is greater than " << MAX_TIME << "!");
			last_time = curr_time;
			r.sleep();
			ros::spinOnce();
			continue;
		}

		// Calculate distance between two coordinates
		double dist = radiusEarth * acos(sin(prevLat) * sin(currLat) + cos(prevLat) * cos(currLat) * cos(prevLon - currLon));  // output in meters
		ROS_DEBUG_STREAM("Distance between points is " << dist);

		// Check for distance to be greater than minimum required, effectively enforcing a min velocity constraint
		if (dist < MIN_DIST)
		{
			ROS_DEBUG_STREAM("Distance between coordinates is " << dist << ", which is smaller than " << MIN_DIST << "!");
			r.sleep();
			ros::spinOnce();
			continue;
		}

		// Calculate heading between two coordinates, taken from here: http://www.sunearthtools.com/tools/distance.php
		ROS_DEBUG("Previous coords are %05.10f, %05.10f", prevLat / toRadians, prevLon / toRadians);
		ROS_DEBUG("Current coords are %05.10f, %05.10f", currLat / toRadians, currLon / toRadians);

		double delPhi = (log(tan( currLat / 2.0 + pi / 4.0) / tan( prevLat / 2.0 + pi / 4.0)));
		double delLon = prevLon - currLon;

		double heading = fmod(atan2( delLon, delPhi ) + 2.0 * pi, 2.0 * pi);
		ROS_DEBUG_STREAM("Heading is " << heading / toRadians);
		double adj_heading = fmod((heading + 1.5707963267949 ), 6.28318530718);

		// Check for robot moving forward or backwards!
		if ((currSpeed < 0) && (prevSpeed < 0))
		{
			ROS_DEBUG_STREAM("Going in reverse!");
			adj_heading = fmod((adj_heading - pi), 6.28318530718);
		} else if (((currSpeed < 0) && (prevSpeed > 0)) || ((currSpeed > 0) && (prevSpeed < 0)))
		{
			ROS_DEBUG_STREAM("Different velocity directions for both points, can't update orientation!");
			r.sleep();
			ros::spinOnce();
			continue;
		}

		ROS_DEBUG_STREAM("Adjusted Heading is " << adj_heading / toRadians);	

		// Populate Imu message
		if (imu_pub.getNumSubscribers() > 0)
		{
			sensor_msgs::Imu imu_msg;

			imu_msg.header.frame_id = "GPS_link";
			imu_msg.header.stamp = ros::Time::now();

			geometry_msgs::Quaternion pose_quat = tf::createQuaternionMsgFromYaw(adj_heading);

			imu_msg.orientation = pose_quat;

			imu_msg.orientation_covariance[0] = 1e6;
			imu_msg.orientation_covariance[4] = 1e6;
			imu_msg.orientation_covariance[8] = 25.0;

			if ((ros::Time::now() - curr_time).toSec() < MAX_TIME)
				{
				imu_pub.publish(imu_msg);
				}
			else 
			{
				ROS_DEBUG_STREAM("Not publishing, heading not updated!");
			}
		}



		// // Populate pose message
		// geometry_msgs::PoseWithCovarianceStamped gpsPose;
		// geometry_msgs::Quaternion pose_quat = tf::createQuaternionMsgFromYaw(adj_heading);

		// gpsPose.header.stamp = curr_time;
		// gpsPose.header.frame_id = "gpsHeading_link";
		// gpsPose.pose.pose.orientation = pose_quat;

		// gpsPose.pose.covariance[0] = 1e6; 
		// gpsPose.pose.covariance[7] = 1e6;
		// gpsPose.pose.covariance[14] = 1e6;
		// gpsPose.pose.covariance[21] = 1e6;
		// gpsPose.pose.covariance[28] = 1e6;
		// gpsPose.pose.covariance[35] = 1e-9;

		// if ((ros::Time::now() - curr_time).toSec() < MAX_TIME)
		// 	{
		// 	pose_pub.publish(gpsPose);
		// 	}
		// else 
		// {
		// 	ROS_INFO_STREAM("Not publishing, heading not updated!");
		// }

		// Update previous time with current and start loop over again
		last_time = curr_time;
		r.sleep();
		ros::spinOnce();

	}
	return 0;

}
