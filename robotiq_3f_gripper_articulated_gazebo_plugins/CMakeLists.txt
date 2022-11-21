cmake_minimum_required(VERSION 3.0.2)
project(robotiq_3f_gripper_articulated_gazebo_plugins)

set(CMAKE_CXX_STANDARD 17)  # ignition gazebo uses C++17

find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins
  robotiq_3f_gripper_articulated_msgs
  roscpp
  sensor_msgs
)

find_package(gazebo REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS
    gazebo_plugins
    robotiq_3f_gripper_articulated_msgs
    roscpp
    sensor_msgs
)

add_library(RobotiqHandPlugin src/RobotiqHandPlugin.cpp)
target_link_libraries(RobotiqHandPlugin PUBLIC ${catkin_LIBRARIES})
add_dependencies(RobotiqHandPlugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS RobotiqHandPlugin DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/${PROJECT_NAME}/plugins/)
