cmake_minimum_required(VERSION 2.8.3)
project(robot_standing_in_line)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  rospy
  gazebo_ros 
  std_msgs
  message_generation
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  TrackerMsg.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

# For move_person_plugin plugin
add_library(move_person_plugin src/move_person.cc)
target_link_libraries(move_person_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

# For move_cup_plugin plugin
add_library(move_cup_plugin src/move_cup.cc)
target_link_libraries(move_cup_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})


catkin_package(
  DEPENDS 
    roscpp 
    rospy
    std_msgs
    gazebo_ros 
    message_runtime
)