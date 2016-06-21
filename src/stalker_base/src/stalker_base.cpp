#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Joy.h"
#include <geometry_msgs/Twist.h>
#include <urdf/model.h>
#include "sensor_msgs/Imu.h"
#include "stalker_hw_interface.h"

#include "lookup.hpp"
#include "group.hpp"
#include "group_command.hpp"

#include <sstream>
#include <string>
#include <vector>
#include <math.h>

#include "controller_manager/controller_manager.h"
#include "ros/callback_queue.h"

#include <boost/chrono.hpp>

typedef boost::chrono::steady_clock time_source;

/**
 * A function to publish an IMU msg
 */
void send_imu_message(const hebi::Feedback& fbk, const ros::Publisher& pub, const std::string &jointName)
{
  // NOTE: We set first cov element to -1 if doesn't exist per message docs
  // ( http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html )
 
  sensor_msgs::Imu msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = jointName;

  // Accelerometers
  if (fbk.imu().accelerometer().has())
  {
    hebi::Vector3f accel = fbk.imu().accelerometer().get();
    msg.linear_acceleration.x = accel.getX();
    msg.linear_acceleration.y = accel.getY();
    msg.linear_acceleration.z = accel.getZ();
  }
  else
  {
    msg.linear_acceleration_covariance[0] = -1;
  }
  // Gyros
  if (fbk.imu().gyro().has())
  {
    hebi::Vector3f gyro = fbk.imu().gyro().get();
    msg.angular_velocity.x = gyro.getX();
    msg.angular_velocity.y = gyro.getY();
    msg.angular_velocity.z = gyro.getZ();
  }
  else
  {
    msg.angular_velocity_covariance[0] = -1;
  }
  // Orientation
  msg.orientation_covariance[0] = -1;
  // Publish
  pub.publish(msg);
}

// Global 'group' pointer so we can access this in a callback...ugly, but until
// we class-ify the node this will work.
hebi::Group* group_g = NULL;
  // hebi::Group* group_g = new hebi::Group();
  
  
/**
* Control loop for Stalker, not realtime safe
*/
void controlLoop(stalker_base::StalkerHardware &stalker,
                 controller_manager::ControllerManager &cm,
                 time_source::time_point &last_time)
{
  /* TODO: Test std::unique_ptr */
  // std::cout << "Hello Tim." << std::endl; 
  // // TODO: Quit program.
  // exit(EXIT_FAILURE);

  // Calculate monotonic time difference
  time_source::time_point this_time = time_source::now();
  boost::chrono::duration<double> elapsed_duration = this_time - last_time;
  ros::Duration elapsed(elapsed_duration.count());
  last_time = this_time;

  // Process control loop
  stalker.updateJointsFromHardware();
  cm.update(ros::Time::now(), elapsed);
  std::cout << "got here in controlLoop!!" << std::endl;
  stalker.writeCommandsToHardware();
  std::cout << "didn't get here in controlLoop!!" << std::endl;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "stalker_base");

  ros::NodeHandle nh;

  // Initialize robot hardware and link to controller manager
  stalker_base::StalkerHardware stalker(nh); 
  controller_manager::ControllerManager cm(&stalker, nh);

  // std::vector<std::string> joint_names = {"Front_Left_Drive", "Rear_Left_Drive", "Front_Right_Drive", "Rear_Right_Drive"};
  // std::vector<std::string> family_names = {"BOOST", "BOOST", "BOOST", "BOOST"};

  // // // Get the lookup and wait to populate (TODO: check for null?)
  // hebi::Lookup lookup;
  // sleep(2);
  // lookup.printTable();

  // // // Get the group
  // for (int i = 0; i < joint_names.size(); i++)
  // {
  //   std::cout << "looking for: " << std::endl;
  //   std::cout << joint_names[i] << std::endl;
  //   std::cout << family_names[i] << std::endl;
  // }
  // std::unique_ptr<hebi::Group> group(lookup.getGroupFromNames(joint_names, family_names, 1000));
  // if (!group)
  // {
  //   ROS_INFO("Could not find modules on network! Quitting!");
  //   return -1;
  // }
  // // THIS IS A HACK to get around limited callback options for ROS subscribe call and the lack of a class for this node.
  // group_g = group.get();
  // // std::unique_ptr<hebi::Group> group_g(lookup.getGroupFromNames(joint_names, family_names, 1000));

  // std::cout << "Found modules!" << std::endl;
  // ROS_INFO("Found modules!");

  // hebi::GroupFeedback fbk(group->size());

  double period_s = 0.04;

  while (ros::ok())
  {
    time_source::time_point last_time = time_source::now();
    controlLoop(stalker, cm, last_time);

    usleep(period_s * 1000000);
    ros::spinOnce();
  }

  // Stop the async callback before returning and deleting objects.
  // group->clearFeedbackHandlers();

  sleep(1); // prevent segfaults? (TODO: needed?)

  return 0;
}