/**
*
*  \author     Tim Mueller-Sim <tmuellersim@cmu.edu>
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted.
*
*/

#ifndef STALKER_HW_INTERFACE_H
#define STALKER_HW_INTERFACE_H

#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/robot_hw.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <string>
#include "lookup.hpp"
#include "group.hpp"
#include "group_command.hpp"

namespace stalker_base
{

  /**
  * Class representing Stalker hardware, allows for ros_control to modify internal state via joint interfaces
  */
  class StalkerHardware :
    public hardware_interface::RobotHW
  {
  public:
    // StalkerHardware(ros::NodeHandle nh, ros::NodeHandle private_nh);
    StalkerHardware(ros::NodeHandle nh);

    void updateJointsFromHardware();

    void writeCommandsToHardware();

  private:

    void registerControlInterfaces();

    ros::NodeHandle nh_;

    // Global 'group' pointer so we can access this in a callback...ugly, but until
    // we class-ify the node this will work.
    hebi::Group* group_g = NULL;

    std::unique_ptr<hebi::Group> group;

    // hebi::GroupFeedback fbk;

    // ROS Control interfaces
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::VelocityJointInterface velocity_joint_interface_;

    // ROS Parameters

    /**
    * Joint structure that is hooked to ros_control's InterfaceManager, to allow control via diff_drive_controller
    */
    struct Joint
    {
      double position;
      double velocity;
      double effort;
      double velocity_command;

      Joint() :
        position(0), velocity(0), effort(0), velocity_command(0)
      { }
    } joints_[4];
  };

}  // namespace stalker
#endif  //STALKER_HW_INTERFACE_H
