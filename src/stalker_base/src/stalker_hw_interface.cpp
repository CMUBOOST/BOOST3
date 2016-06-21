/**
*
*  \author     Tim Mueller-Sim <tmuellersim@cmu.edu>
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted.
*
*/

#include "stalker_hw_interface.h"
#include <boost/assign/list_of.hpp>

#include "lookup.hpp"
#include "group.hpp"
#include "group_command.hpp"
#include "sensor_msgs/Imu.h"

#include <sstream>
// #include <string>
#include <vector>
#include <math.h>

namespace stalker_base
{

  /**
  * Initialize Stalker hardware
  */
    StalkerHardware::StalkerHardware(ros::NodeHandle nh)
    :
    nh_(nh)
  {
    registerControlInterfaces();
  }

  /**
  * Register interfaces with the RobotHW interface manager, allowing ros_control operation
  */
  void StalkerHardware::registerControlInterfaces()
  {
    std::vector<std::string> joint_names = {"Front_Left_Drive", "Rear_Left_Drive", "Front_Right_Drive", "Rear_Right_Drive"};

    // ros::V_string joint_names = boost::assign::list_of("Front_Left_Drive")
      // ("Rear_Left_Drive")("Front_Right_Drive")("Rear_Right_Drive");
    for (unsigned int i = 0; i < joint_names.size(); i++)
    {
      hardware_interface::JointStateHandle joint_state_handle(joint_names[i],
                                                              &joints_[i].position, &joints_[i].velocity,
                                                              &joints_[i].effort);
      joint_state_interface_.registerHandle(joint_state_handle);

      hardware_interface::JointHandle joint_handle(
        joint_state_handle, &joints_[i].velocity_command);
      velocity_joint_interface_.registerHandle(joint_handle);
    }
    registerInterface(&joint_state_interface_);
    registerInterface(&velocity_joint_interface_);

    // std::vector<std::string> joint_names = {"Front_Left_Drive", "Rear_Left_Drive", "Front_Right_Drive", "Rear_Right_Drive"};
    std::vector<std::string> family_names = {"BOOST", "BOOST", "BOOST", "BOOST"};

    // Get the lookup and wait to populate (TODO: check for null?)
    hebi::Lookup lookup;
    sleep(2);
    lookup.printTable();

    // Get the group
    for (int i = 0; i < joint_names.size(); i++)
    {
      std::cout << "looking for: " << std::endl;
      std::cout << joint_names[i] << std::endl;
      std::cout << family_names[i] << std::endl;
    }
    std::unique_ptr<hebi::Group> group(lookup.getGroupFromNames(joint_names, family_names, 1000));
    if (!group)
    {
      ROS_INFO("Could not find modules on network! Quitting!");
      exit(EXIT_FAILURE);
    }
    // THIS IS A HACK to get around limited callback options for ROS subscribe call and the lack of a class for this node.
    group_g = group.get();
    // std::unique_ptr<hebi::Group> group_g(lookup.getGroupFromNames(joint_names, family_names, 1000));

    std::cout << "Found modules!" << std::endl;
    ROS_INFO("Found modules!");

    hebi::GroupFeedback fbk(group->size());
    
    // group->requestFeedback(&fbk, 1000);
  }

  /**
  * Pull latest speed and travel measurements and store in joint structure for ros_control
  */
  void StalkerHardware::updateJointsFromHardware()
  {
    std::cout << "got here in updateJointsFromHardware!!" << std::endl;
    hebi::GroupFeedback fbk(4);
    if (group_g->requestFeedback(&fbk, 1000))
    {
      for (int i = 0; i < 4; i++)
      {
        if ((i ==0) || (i == 1))
        {
          joints_[i].position = -fbk[i].actuator().position().get();
          joints_[i].velocity = -fbk[i].actuator().velocity().get();
        }
        else 
        {
          joints_[i].position = fbk[i].actuator().position().get();
          joints_[i].velocity = fbk[i].actuator().velocity().get();
        }
      }
    }
    else
    {
      std::cout << "feedback timeout!" << std::endl;
    }
  }

  /**
  * Get latest velocity commands from ros_control via joint structure, and send to motor drivers
  */
  void StalkerHardware::writeCommandsToHardware()
  {
    hebi::GroupCommand cmd(4);
    cmd[0].actuator().velocity().set(-joints_[0].velocity_command);
    cmd[1].actuator().velocity().set(-joints_[1].velocity_command);
    cmd[2].actuator().velocity().set(joints_[2].velocity_command);
    cmd[3].actuator().velocity().set(joints_[3].velocity_command);
    std::cout << "got here in writeCommandsToHardware!!" << std::endl;
    group_g->sendCommand(cmd);
    std::cout << "got here in writeCommandsToHardware!!" << std::endl;
  }

}  // namespace stalker_base
