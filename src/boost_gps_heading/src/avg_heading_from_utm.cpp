#include "ros/ros.h"
#include "std_msgs/Float32.h"

#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

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
const double pi = 4*atan(1);
const double toRadians = pi / 180.0;
double prevX = 0.0;
double prevY = 0.0;
double currX = 0.0;
double currY = 0.0;
double prevTime = 0.0;
double currTime = 0.0;
double prevSpeed = 0.0;
double currSpeed = 0.0;
int sample_size = 60;  // 20
float pubRate = 20;  // 20
double ang_velocity; 


typedef struct list_node list;
struct list_node {
    double x;
    double y;
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

void enq(queue *Q, double x, double y, double time) {
// add a node to the end of the queue (linked list)
    list *l = new list;
    l->next = NULL;
    l->x = x;
    l->y = y;
    l->time = time;
    if (Q->size == 0) Q->front = l;
    else Q->back->next = l;
    Q->back = l;
    Q->size++;
    return;
}

double bestFit(queue *Q) {
    bool posX;
    bool posY; 
    double sumX = 0.0; //sum of all x points
    double sumY = 0.0; //sum of all y points
    double sumXY = 0.0; //sum of the product of each x and y pairing
    double sumXX = 0.0; //sum of the x points squared
    double n = Q->size;
    list *cur = Q->front;
    for (int i = 0; i < n; i++) {
        sumX += cur->x;
        sumY += cur->y;
        sumXY += cur->x * cur->y;
        sumXX += cur->x * cur->x;
        cur = cur->next;
    }
    list* front = Q->front;
    list* back = Q->back;
    // formula for the slope of best fit line
    double slope = (n*sumXY - sumX*sumY)/(n*sumXX - sumX*sumX); 
    double heading = atan(slope);
    
    if (fabs(slope) < 1) { // moving more in x direction
        if ((back->x - front->x) > 0) posX = true; // moving in positive x direction
        else posX = false; // moving in negative x direction
    	//adjust heading angle
	if ((posX == true) && (heading < 0)) heading = heading + 2*pi; 
        else if (posX == false) heading = heading + pi;   
    }
    else { // moving more in y direction
        if ((back->y - front->y) > 0) posY = true; // moving in positive y direction
        else posY = false; // moving in negative y direction
        //adjust heading angle 
        if ((posY == true) && (heading < 0)) heading = heading + pi;
        else if ((posY == false) && (heading < 0)) heading = heading + 2*pi;
	else if ((posY == false) && (heading >= 0)) heading = heading + pi;  
    }
        
    return heading;
}

double averageHeading(queue *Q) {
    // note size of queue must be larger than 1
    double avg_heading = 0.0;
    double dx;
    double dy;
    double curr_x, curr_y, prev_x, prev_y;
    double heading;
    list *prev = Q->front;
    list *cur = prev->next;
    for (int i = 1; i < Q->size; i++) {
        curr_x = cur->x;
        curr_y = cur->y;
        prev_x = prev->x;
        prev_y = prev->y;
	 
        dx = curr_x - prev_x;
		dy = curr_y - prev_y;
        heading = fmod(atan2(dy, dx) + 2.0 * pi, 2.0 * pi);
        avg_heading += heading;
        prev = prev->next;
        cur = cur->next;
    }
    avg_heading = avg_heading / (Q->size - 1);    
    return avg_heading;
}    

// TODO: ADD ALTITUDE TO GET 3D POSE ESTIMATE

void gpsHeadingCallback(const nav_msgs::Odometry::ConstPtr& utm_msg, const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& twist_msg)
{
	ROS_DEBUG_STREAM("Got into callback!");
	
	ang_velocity = twist_msg->twist.twist.angular.z;	
	
	// Add x and y to respective arrays, along with time, check for GBAS and fix
	if (fabs(twist_msg->twist.twist.linear.x) > MIN_SPEED)
	{
		prevX = currX;
		prevY = currY;
		prevTime = currTime;
		prevSpeed = currSpeed;

		currX = utm_msg->pose.pose.position.x;
		currY = utm_msg->pose.pose.position.y;
		currTime = utm_msg->header.stamp.sec + (1e-9 * utm_msg->header.stamp.nsec);

		currSpeed = twist_msg->twist.twist.linear.x;

		return;
	} 
	else 
	{
		ROS_DEBUG_STREAM("GBAS is moving too slow!  Linear Speed: " << twist_msg->twist.twist.linear.x);
		return;
	}
}




int main(int argc, char **argv)
{

	const double MIN_DIST = 0.005; // meters, needs to be at least 3 times the standard deviation of the GPS signal
	const double MAX_TIME = 5; // seconds

	ros::init(argc, argv, "boost_avg_heading_from_gps_publisher");

	// Use message_filters to combine and sync two topics
	ros::NodeHandle nh;
	message_filters::Subscriber<nav_msgs::Odometry> utm_sub(nh, "odometry/utm", 1);
	message_filters::Subscriber<geometry_msgs::TwistWithCovarianceStamped> twist_sub(nh, "base/cmd_vel", 1);

	typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, geometry_msgs::TwistWithCovarianceStamped> MySyncPolicy;
	// ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), utm_sub, twist_sub);
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

	double xCheck = currX;
	double yCheck = currY;
	double avg_heading = 0;
	double fit_heading = 0;	
	
    	// Create Queue header for position data
    	queue *posQ = new queue;
    	posQ->size = 0;
    	posQ->front = NULL;
    	posQ->back = NULL;

	ros::Rate r(pubRate);
	while (ros::ok())
	{
		
		// Check that positions were updated
		if (xCheck == currX || yCheck == currY)
		{
			ROS_DEBUG_STREAM(" X/Y have not been updated!");

			r.sleep();
			ros::spinOnce();
			continue;
		}
		double xCheck = currX;
		double yCheck = currY;

		double dt = currTime - prevTime;

		if (dt > MAX_TIME)
		{
			ROS_DEBUG_STREAM("Time between coordinates is " << dt << ", which is greater than " << MAX_TIME << "!");

			r.sleep();
			ros::spinOnce();
			continue;
		}

		// Calculate distance between two coordinates
		double dx = currX-prevX;
		double dy = currY-prevY;
		double dist = pow( dx*dx + dy*dy, 0.5);  // output in meters
		ROS_DEBUG("dist is: %05.16f", dist);

		// Check for distance to be greater than minimum required, effectively enforcing a min velocity constraint
		if (dist < MIN_DIST)
		{
			ROS_DEBUG("Distance between coordinates is %05.10f, which is smaller than %05.10f, dt is %05.10f!", dist, MIN_DIST, dt);
			ROS_DEBUG("Previous Location is: x: %05.10f, y: %05.10f", prevX, prevY);
			ROS_DEBUG("Current Location is:  x: %05.10f, y: %05.10f", currX, currY);

			r.sleep();
			ros::spinOnce();
			continue;
		}

        	// Enqueue new position data onto the position queue
        	enq(posQ, currX, currY, currTime);
        	if (posQ->size > sample_size) deq(posQ);

        	// Loop through the queue and print the positions within
        	// ROS_DEBUG_STREAM("CURRENT POSITION QUEUE:");
        	// list *cur = posQ->front;
        	// for (int j = 1; j <= posQ->size; j++) {
        	//     //ROS_INFO_STREAM(j << ":  x: " << cur->x << " y: " << cur->y << " time: " << cur->time);
        	//     // printf("%d:  x: %05.10f  y: %05.10f\n", j, cur->x, cur->y);
        	//     cur = cur->next;
        	// }

        	// Calculate Average Heading using a line of best fit
        	// double slope = bestFit(posQ);
        	// ROS_INFO_STREAM("slope = " << slope << "\n");
        
		ROS_DEBUG("Previous Location is: x: %05.10f, y: %05.10f", prevX, prevY);
		ROS_DEBUG("Current Location is:  x: %05.10f, y: %05.10f", currX, prevY);

		double heading = fmod(atan2(dy, dx) + 2.0*pi, 2.0*pi);
		ROS_DEBUG_STREAM("Heading is " << heading / toRadians);

        	// Calculate Average Heading from position Queue (simple average)
        	if (posQ->size == sample_size) {
			avg_heading = averageHeading(posQ);
			fit_heading = bestFit(posQ);
		}

        	ROS_DEBUG_STREAM("Average Heading is  " << avg_heading / toRadians);
    		ROS_DEBUG_STREAM("Line Fit Heading is " << fit_heading / toRadians);
		// Check for robot moving forward or backwards!
		if ((currSpeed < 0) && (prevSpeed < 0))
		{
			ROS_DEBUG_STREAM("Going in reverse!");
			avg_heading = fmod((avg_heading - pi), 2.0*pi);
			fit_heading = fmod((fit_heading - pi), 2.0*pi);
		} else if (((currSpeed < 0) && (prevSpeed > 0)) || ((currSpeed > 0) && (prevSpeed < 0)))
		{
			ROS_DEBUG_STREAM("Different velocity directions for both points, can't update orientation!");

			r.sleep();
			ros::spinOnce();
			continue;
		}
		 
		// NEWEST ADDTION
		// DELETE ALL NODES IN QUEUE IF TWIST IS LARGER THAN GIVEN VALUE
		if (ang_velocity > 0.2) { 
			ROS_DEBUG_STREAM("Angular Velocity is too large... Deleting Queue");
			// delete queue to reset heading average
			while (posQ->size > 0) {
				deq(posQ);
			}
		} 
				
		
		// Populate Imu message
		if (posQ->size == sample_size)
		{
			sensor_msgs::Imu imu_msg;

			imu_msg.header.frame_id = "GPS_link";
			imu_msg.header.stamp = ros::Time::now();

			geometry_msgs::Quaternion pose_quat = tf::createQuaternionMsgFromYaw(fit_heading);

			imu_msg.orientation = pose_quat;

			imu_msg.orientation_covariance[0] = 1e6;
			imu_msg.orientation_covariance[4] = 1e6;
			imu_msg.orientation_covariance[8] = 1e-2;

			if (fabs(ros::Time::now().toSec() - currTime) < MAX_TIME)
				{
				imu_pub.publish(imu_msg);

				ROS_DEBUG_STREAM("Publishing IMU message on /gps/imu/data!");
				ROS_DEBUG_STREAM("CURRENT POSITION QUEUE:");
				list *cur2 = posQ->front;
				
				ROS_DEBUG_STREAM("posQ size is " << posQ->size);
				for (int j = 1; j <= posQ->size; j++) 
				{
					ROS_DEBUG("%d:  x: %05.10f  y: %05.10f", j, cur2->x, cur2->y);
					ROS_DEBUG_STREAM("Time: " << cur2->time);
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
				ROS_DEBUG_STREAM("Not publishing, heading not updated!" << fabs(ros::Time::now().toSec() - currTime));
			}
		}

		// // Populate pose message
		// geometry_msgs::PoseWithCovarianceStamped gpsPose;
		// geometry_msgs::Quaternion pose_quat = tf::createQuaternionMsgFromYaw(avg_heading);

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


	ROS_INFO_STREAM("ROS not OK!");

	
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
