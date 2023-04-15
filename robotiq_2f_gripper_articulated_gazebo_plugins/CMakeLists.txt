cmake_minimum_required(VERSION 3.0.2)
project(robotiq_2f_gripper_articulated_gazebo_plugins)

set(CMAKE_CXX_STANDARD 17)  # ignition gazebo uses C++17

find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins
  robotiq_2f_gripper_control # for msg
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
    robotiq_2f_gripper_control # for msg
    roscpp
    sensor_msgs
)

add_library(robotiq_2f_gripper_plugin src/RobotiqHandPlugin.cpp)
target_link_libraries(robotiq_2f_gripper_plugin PUBLIC ${catkin_LIBRARIES})
add_dependencies(robotiq_2f_gripper_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS robotiq_2f_gripper_plugin DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/${PROJECT_NAME}/plugins/)
