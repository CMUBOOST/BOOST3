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

/* TODO 
		- USE TWIST TO ENABLE LARGE CHANGES OF CALCULATED HEAD (BYPASSING LOW-PASS FILTER), STORE Z-ORIENTATION TWIST VALUE IN QUEUE NODE
		- WRITE LOW PASS FILTER FOR CHANGES IN COURSE HEADING, STORE PREVIOUS HEADING SOMEWHERE
*/

const double MIN_SPEED = 0.1; // m/s
const double toRadians = 3.14159265359 / 180.0;
const double pi = 3.14159265359;
double prevLat = 0.0;
double prevLon = 0.0;
double currLat = 0.0;
double currLon = 0.0;
double prevTime = 0.0;
double currTime = 0.0;
double prevSpeed = 0.0;
double currSpeed = 0.0;
int sample_size = 10;

// ros::Time curr_time;
// ros::Time last_time;

typedef struct list_node list;
struct list_node {
    double lat;
    double lon;
    double time;
    list *next;
};

typedef struct queue_header queue;
struct queue_header {
    list *front;
    list *back;
    int size;
};

void deq(queue *Q) {
// remove the first node of the queue (linked list)
    list *front_ptr = Q->front;
    if (Q->size == 1) Q->front == NULL;
    else Q->front = Q->front->next;
    Q->size--;
    delete front_ptr;
    return;
}

void enq(queue *Q, double lat, double lon, double time) {
// add a node to the end of the queue (linked list)
    list *l = new list;
    l->next = NULL;
    l->lat = lat;
    l->lon = lon;
    l->time = time;
    if (Q->size == 0) Q->front = l;
    else Q->back->next = l;
    Q->back = l;
    Q->size++;
    return;
}

double bestFit(queue *Q) {
    double sumX = 0.0; //sum of all x points
    double sumY = 0.0; //sum of all y points
    double sumXY = 0.0; //sum of the product of each x and y pairing
    double sumXX = 0.0; //sum of the x points squared
    double n = Q->size;
    list *cur = Q->front;
    for (int i = 0; i < n; i++) {
        sumX += cur->lat;
        sumY += cur->lon;
        sumXY += cur->lat * cur->lon;
        sumXX += cur->lat * cur->lat;
        cur = cur->next;
    }
    // formula for the slope of best fit line
    double slope = (n*sumXY - sumX*sumY)/(n*sumXX - sumX*sumX); 
    return slope;
}

double averageHeading(queue *Q) {
    // note size of queue must be larger than 1
    double avg_heading = 0.0;
    double delPhi;
    double delLon;
    double heading;
    list *prev = Q->front;
    list *cur = prev->next;
    for (int i = 1; i < Q->size; i++) {
        currLat = cur->lat;
        currLon = cur->lon;
        prevLat = prev->lat;
        prevLon = prev->lon; 
        delPhi = (log(tan( currLat / 2.0 + pi / 4.0) / tan( prevLat / 2.0 + pi / 4.0)));
        delLon = prevLon - currLon;
        heading = fmod(atan2( delLon, delPhi ) + 2.0 * pi, 2.0 * pi);
        avg_heading += heading;
        prev = prev->next;
        cur = cur->next;
    }
    double avg_adj_heading = avg_heading / (Q->size - 1);
	avg_adj_heading = fmod((avg_adj_heading + 1.5707963267949 ), 6.28318530718);    
    return avg_adj_heading;
}    

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

		// last_time = curr_time;
		// curr_time = ros::Time::now();
		return;
	} 
	else 
	{
		ROS_DEBUG_STREAM("GBAS is not available or moving too slow!  GPS Status: " << fix_msg->status.status << "   Linear Speed: " << twist_msg->twist.twist.linear.x);
		return;
	}
}


int main(int argc, char **argv)
{

	const double MIN_DIST = 0.005; // meters, needs to be at least 3 times the standard deviation of the GPS signal
	const double MAX_TIME = 5; // seconds
	const double radiusEarth = 6372795.477598;  // Radius of earth in meters

	ros::init(argc, argv, "boost_avg_heading_from_gps_publisher");

	// Use message_filters to combine and sync two topics
	ros::NodeHandle nh;
	message_filters::Subscriber<sensor_msgs::NavSatFix> fix_sub(nh, "fix", 1);
	message_filters::Subscriber<geometry_msgs::TwistWithCovarianceStamped> twist_sub(nh, "base/cmd_vel", 1);

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::NavSatFix, geometry_msgs::TwistWithCovarianceStamped> MySyncPolicy;

	// ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), fix_sub, twist_sub);
	sync.registerCallback(boost::bind(&gpsHeadingCallback, _1, _2));

	// Initialize pose publisher, which will hold the heading information calculated in the callBack
	// ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("gpsHeading", 1);
	ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("gps/imu/data", 1, false);

	prevTime = ros::Time::now().toSec();

	// ros::Subscriber sub_fix = nh.subscribe("fix", 1, gpsHeadingCallback);

	// Sleep to allow time for fix to be updated, HACK- FIX LATER
	sleep(1);
	ros::spinOnce();
	sleep(0.5);
	ros::spinOnce();  // Fill in currLat

	double latCheck = currLat;
	double lonCheck = currLon;
    double avg_adj_heading = 0;

    // Create Queue header for position data
    queue *posQ = new queue;
    posQ->size = 0;
    posQ->front = NULL;
    posQ->back = NULL;

	ros::Rate r(20);
	while (ros::ok())
	{

		// Check that lat and long were updated
		if (latCheck == currLat || lonCheck == currLon)
		{
			ROS_DEBUG_STREAM(" Lat/Lons have not been updated!");

			r.sleep();
			ros::spinOnce();
			continue;
		}
		double latCheck = currLat;
		double lonCheck = currLon;

		// Latitudes and longitudes are in radians at this point!
		double dt = currTime - prevTime;

		if (dt > MAX_TIME)
		{
			ROS_DEBUG_STREAM("Time between coordinates is " << dt << ", which is greater than " << MAX_TIME << "!");

			r.sleep();
			ros::spinOnce();
			continue;
		}

		// Calculate distance between two coordinates
		double dist = radiusEarth * acos(sin(prevLat) * sin(currLat) + cos(prevLat) * cos(currLat) * cos(prevLon - currLon));  // output in meters
		ROS_DEBUG("dist is: %05.16f", dist);

		// Check for distance to be greater than minimum required, effectively enforcing a min velocity constraint
		if (dist < MIN_DIST)
		{
			ROS_DEBUG("Distance between coordinates is %05.10f, which is smaller than %05.10f, dt is %05.10f!", dist, MIN_DIST, dt);
			ROS_DEBUG("Previous coords are %05.10f, %05.10f", prevLat / toRadians, prevLon / toRadians);
			ROS_DEBUG("Current coords are %05.10f, %05.10f", currLat / toRadians, currLon / toRadians);

			r.sleep();
			ros::spinOnce();
			continue;
		}

        // Enqueue new position data onto the position queue
        enq(posQ, currLat, currLon, currTime);
        if (posQ->size > sample_size) deq(posQ);

        // Loop through the queue and print the positions within
        // ROS_DEBUG_STREAM("CURRENT POSITION QUEUE:");
        // list *cur = posQ->front;
        // for (int j = 1; j <= posQ->size; j++) {
        //     //ROS_INFO_STREAM(j << ":  lat: " << cur->lat << " lon: " << cur->lon << " time: " << cur->time);
        //     // printf("%d:  lat: %05.10f  lon: %05.10f\n", j, cur->lat/toRadians, cur->lon/toRadians);
        //     cur = cur->next;
        // }

        // Calculate Average Heading using a line of best fit
        // double slope = bestFit(posQ);
        // ROS_INFO_STREAM("slope = " << slope << "\n");
        
		// Calculate heading between two coordinates, taken from here: http://www.sunearthtools.com/tools/distance.php
		ROS_DEBUG("Previous coords are %05.10f, %05.10f", prevLat / toRadians, prevLon / toRadians);
		ROS_DEBUG("Current coords are %05.10f, %05.10f", currLat / toRadians, currLon / toRadians);

		double delPhi = (log(tan( currLat / 2.0 + pi / 4.0) / tan( prevLat / 2.0 + pi / 4.0)));
		double delLon = prevLon - currLon;

		double heading = fmod(atan2( delLon, delPhi ) + 2.0 * pi, 2.0 * pi);
		ROS_DEBUG_STREAM("Heading is " << heading / toRadians);
		double adj_heading = fmod((heading + 1.5707963267949 ), 6.28318530718);
		ROS_DEBUG_STREAM("Adjusted Heading is " << adj_heading / toRadians);

        // Calculate Average Heading from position Queue (simple average)
        if (posQ->size == sample_size) avg_adj_heading = averageHeading(posQ);

        ROS_DEBUG_STREAM("Average Adj Heading is " << avg_adj_heading / toRadians);
    
		// Check for robot moving forward or backwards!
		if ((currSpeed < 0) && (prevSpeed < 0))
		{
			ROS_DEBUG_STREAM("Going in reverse!");
			avg_adj_heading = fmod((avg_adj_heading - pi), 6.28318530718);
		} else if (((currSpeed < 0) && (prevSpeed > 0)) || ((currSpeed > 0) && (prevSpeed < 0)))
		{
			ROS_INFO_STREAM("Different velocity directions for both points, can't update orientation!");

			r.sleep();
			ros::spinOnce();
			continue;
		}


		// Populate Imu message
		if (posQ->size == sample_size)
		{
			sensor_msgs::Imu imu_msg;

			imu_msg.header.frame_id = "GPS_link";
			imu_msg.header.stamp = ros::Time::now();

			geometry_msgs::Quaternion pose_quat = tf::createQuaternionMsgFromYaw(avg_adj_heading);

			imu_msg.orientation = pose_quat;

			imu_msg.orientation_covariance[0] = 1e6;
			imu_msg.orientation_covariance[4] = 1e6;
			imu_msg.orientation_covariance[8] = 1e-2;

			if (fabs(ros::Time::now().toSec() - currTime) < MAX_TIME)
				{
				imu_pub.publish(imu_msg);

				ROS_INFO_STREAM("CURRENT POSITION QUEUE:");
				list *cur2 = posQ->front;
				
				ROS_INFO_STREAM("posQ size is " << posQ->size);
				for (int j = 1; j <= posQ->size; j++) 
				{
					printf("%d:  lat: %05.10f  lon: %05.10f", j, cur2->lat/toRadians, cur2->lon/toRadians);
					ROS_INFO_STREAM("Time: " << cur2->time);
					cur2 = cur2->next;
        		}

    //     		ROS_INFO_STREAM("DELETING QUEUE!");

				// // Free memory from the position queue
				// list *cur = posQ->front;
				// for (int i = 0; i < posQ->size; i++) 
				// {
				// 	ROS_INFO_STREAM("\tdeleting node number: " << i);
				// 	list *temp = cur;
				// 	cur = cur->next;
				// 	delete temp;
				// }
				
				// delete posQ;

				// // Create Queue header for position data
				// queue *posQ = new queue;
				// posQ->size = 0;
				// posQ->front = NULL;
				// posQ->back = NULL;



				}
			else 
			{
				ROS_INFO_STREAM("Not publishing, heading not updated!");
			}
		}

		// // Populate pose message
		// geometry_msgs::PoseWithCovarianceStamped gpsPose;
		// geometry_msgs::Quaternion pose_quat = tf::createQuaternionMsgFromYaw(avg_adj_heading);

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
		r.sleep();
		ros::spinOnce();

	}
    // Free memory from the position queue
    list *cur = posQ->front;
    for (int i = 0; i < posQ->size; i++) {
        list *temp = cur;
        cur = cur->next;
        delete temp;
    }
    delete posQ;
	return 0;

}
