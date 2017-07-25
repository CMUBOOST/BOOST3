cmake_minimum_required(VERSION 2.8.3)
project(boost_gps_heading)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  nav_msgs
  message_filters
  std_srvs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED)


################################################
## Declare ROS messages, services and actions ##
################################################



################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(

)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
	include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a C++ executable
# add_executable(boost_gps_heading_node src/heading_from_gps.cpp)
# add_executable(boost_gps_avg_heading_node src/avg_heading_from_gps.cpp)
# add_executable(boost_utm_avg_heading_node src/avg_heading_from_utm.cpp)
# add_executable(boost_utm_avg_heading_service_node src/avg_heading_from_utm_service.cpp)
add_executable(boost_gps_imu_publisher src/gps_imu_publisher.cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(boost_gps_heading_node
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
# )

# target_link_libraries(boost_gps_avg_heading_node
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
# )

# target_link_libraries(boost_utm_avg_heading_node
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
# )

# target_link_libraries(boost_utm_avg_heading_service_node
#   ${catkin_LIBRARIES}
# )

target_link_libraries(boost_gps_imu_publisher
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

#############
## Install ##
#############

#############
## Testing ##
#############


