cmake_minimum_required(VERSION 2.8.3)
project(cartesian_trajectory_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  cartesian_trajectory_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  eigen_conversions
  std_srvs
  controller_manager_msgs
  roslib)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    actionlib_msgs
    cartesian_trajectory_msgs
    tf2_ros
    tf2_eigen
    std_srvs
    controller_manager_msgs
  DEPENDS
    EIGEN3
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} )

add_executable(compliance_trajectory_server src/compliance_trajectory_server.cpp)

target_include_directories(compliance_trajectory_server PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS})
target_link_libraries(
  compliance_trajectory_server
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)
