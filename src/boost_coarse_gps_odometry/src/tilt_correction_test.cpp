#include "ros/ros.h"
#include "std_msgs/Float32.h"

#include "nav_msgs/Odometry.h"

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

double PI = 3.14159;
static ros::Publisher odom_pub;
// static tf::TransformBroadcaster odom_broadcaster;
std::string frame_id, child_frame_id;
bool publish_tf;
nav_msgs::Odometry odom;
sensor_msgs::Imu imu;
geometry_msgs::TransformStamped odom_tf;
double gps_height = 1.702; // height of gps receiver in meters
double prev_dy = 0.0;

// TODO - Make a ros param
/*
 * Simpson Fields
 */
double x_datum = 340634.710905;
double y_datum = 3832788.98314;
double z_datum = 222.819;

/*
 * The Cut
 */
// double x_datum = 589611.370198;
// double y_datum = 4477380.87125;
// double z_datum = 257.035;

 /*
  * Pee Dee Fields, Florence SC
  */
// double x_datum = 614797.462364;
// double y_datum = 3797238.64635;
// double z_datum = 7.318;


void coarseOdomCallback(const nav_msgs::Odometry::ConstPtr& orient_msg, const sensor_msgs::Imu::ConstPtr& imu_msg)
{
  double dy_max = 0.01;
	ROS_DEBUG_STREAM("Got into callback!");
  // ROS_INFO_STREAM("Time difference of fix and imu topics is :" << dt);
	
	// trace gps receiver to base link using imu data
	double acc_y = -imu_msg->linear_acceleration.y;
	double acc_z = -imu_msg->linear_acceleration.z;
	double tilt_angle = atan(acc_y / acc_z);
	double dy = gps_height*sin(tilt_angle);

  // if (fabs(dy - prev_dy) > dy_max)
  // {
  //   if (dy < 0) dy = -dy_max;
  //   if (dy > 0) dy = dy_max;
  // }
	ROS_INFO_STREAM("Tilt angle: " << tilt_angle * 180 / PI << "\tdy: " << dy << "\tdelta dy: " << fabs(dy - prev_dy)); 
	
	double quat_x = orient_msg->pose.pose.orientation.x;
	double quat_y = orient_msg->pose.pose.orientation.y;
	double quat_z = orient_msg->pose.pose.orientation.z;
	double quat_w = orient_msg->pose.pose.orientation.w;
	tf::Quaternion q(quat_x, quat_y, quat_z, quat_w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw; // tilt angles
	m.getRPY(roll, pitch, yaw);
	double heading = yaw; 
	double adj_x = dy*sin(heading);
	double adj_y = dy*cos(heading);	
	ROS_INFO_STREAM("Adj X: " << adj_x << "\tAdj Y: " << adj_y << "\n");	


 //  // odom is a global variable
	// odom.header.stamp = trans_msg->header.stamp;
	// odom.header.frame_id = frame_id;
	// odom.child_frame_id = child_frame_id;

	// odom.pose.pose.position.x = (trans_msg->pose.pose.position.x - x_datum) - adj_x;
	// odom.pose.pose.position.y = (trans_msg->pose.pose.position.y - y_datum) + adj_y;
	// odom.pose.pose.position.z = trans_msg->pose.pose.position.z - z_datum;

	// odom.pose.pose.orientation = orient_msg->pose.pose.orientation;

 //    	// Use ENU covariance to build XYZRPY covariance
 //    	boost::array<double, 36> covariance = {{
 //      trans_msg->pose.covariance[0],
 //      trans_msg->pose.covariance[1],
 //      trans_msg->pose.covariance[2],
 //      0, 0, 0,
 //      trans_msg->pose.covariance[6],
 //      trans_msg->pose.covariance[7],
 //      trans_msg->pose.covariance[8],
 //      0, 0, 0,
 //      trans_msg->pose.covariance[12],
 //      trans_msg->pose.covariance[13],
 //      trans_msg->pose.covariance[14],
 //      0, 0, 0,
 //      0, 0, 0, 
 //      orient_msg->pose.covariance[21],
 //      orient_msg->pose.covariance[22],
 //      orient_msg->pose.covariance[23],
 //      0, 0, 0,
 //      orient_msg->pose.covariance[27],
 //      orient_msg->pose.covariance[28],
 //      orient_msg->pose.covariance[29],
 //      0, 0, 0,
 //      orient_msg->pose.covariance[33],
 //      orient_msg->pose.covariance[34],
 //      orient_msg->pose.covariance[35],
 //    }};

 //    odom.pose.covariance = covariance;

 //    odom.twist = orient_msg->twist;

	// ROS_DEBUG_STREAM("Publishing odom!");
 //    odom_pub.publish(odom);


 //    // Publish the transform over tf
 //    // odom_tf is a global variable
 //    odom_tf.header.stamp = ros::Time::now();
 //    odom_tf.header.frame_id = frame_id;
 //    odom_tf.child_frame_id = child_frame_id;

 //    odom_tf.transform.translation.x = (trans_msg->pose.pose.position.x - x_datum) - adj_x;
 //    odom_tf.transform.translation.y = (trans_msg->pose.pose.position.y - y_datum) + adj_y;
 //    odom_tf.transform.translation.z = trans_msg->pose.pose.position.z - z_datum;
 //    ROS_DEBUG("Translation tf is x: %010.10f,  y: %010.10f, z: %010.10f", odom_tf.transform.translation.x, odom_tf.transform.translation.y, odom_tf.transform.translation.z);
 //    odom_tf.transform.rotation = orient_msg->pose.pose.orientation;

    prev_dy = dy;

    // send the transform
	  // ROS_INFO_STREAM("Publishing tf!");
    // odom_broadcaster.sendTransform(odom_tf);

	return;
}




int main(int argc, char **argv)
{

	ros::init(argc, argv, "boost_coarse_gps_odometry_node");

	// Use message_filters to combine and sync two topics
	ros::NodeHandle nh;

	ros::NodeHandle priv_node("~");
	priv_node.param<std::string>("frame_id", frame_id, "odom/coarse_gps");
	priv_node.param<std::string>("child_frame_id", child_frame_id, "base_link");
  	priv_node.param<bool>("publish_tf", publish_tf, true);


	message_filters::Subscriber<nav_msgs::Odometry> orient_sub(nh, "odometry/filtered_imu_encoders", 10);
	message_filters::Subscriber<sensor_msgs::Imu> imu_sub(nh, "mti/sensor/imu", 10);
	
	typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::Imu> MySyncPolicy;

	// ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), orient_sub, imu_sub);
	sync.registerCallback(boost::bind(&coarseOdomCallback, _1, _2));

	// Initialize odom publisher, which will hold the pose information
	// ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odometry/coarse_gps", 1, false);
	odom_pub = nh.advertise<nav_msgs::Odometry>("odometry/coarse_gps", 1, false);

	tf::TransformBroadcaster odom_broadcaster;

	ros::Time last_time = ros::Time::now();

	ros::Rate r(20.0);
	while (ros::ok())
	{

		last_time = odom.header.stamp;
		ros::spinOnce();   
		// Check for not same topic

    	if (last_time != odom.header.stamp)
    	{
	    	// perform subscribed callback again and wait until it has to publish again
		    ROS_DEBUG_STREAM("Publishing odom!");
	    	odom_pub.publish(odom);

        if (publish_tf)
        {
          ROS_DEBUG_STREAM("Publishing tf!");
          odom_broadcaster.sendTransform(odom_tf);
        }
    	}
    	else
    	{
    		ROS_DEBUG_STREAM("Same odometry/utm topic as before!");
    	}

		// ros::spinOnce();
		r.sleep();
	}

	return 0;
}
