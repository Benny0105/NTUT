cmake_minimum_required(VERSION 3.0.2)
project(ekf)

## Compile as C++11
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  nav_msgs
  geometry_msgs
  message_generation
  gazebo_msgs
)

## Find Eigen
find_package(Eigen3 REQUIRED)

###################################
## Declare ROS messages/services ##
###################################
# 如果有自定义 msg/srv，放在这里

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    rospy
    sensor_msgs
    std_msgs
    tf
    nav_msgs
    geometry_msgs
    message_runtime
    gazebo_msgs
  DEPENDS EIGEN3
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## EKF 主库
add_library(${PROJECT_NAME}
  src/px4_style_ekf.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

## px4_style_ekf_node 可执行
add_executable(px4_style_ekf_node src/px4_style_ekf_node.cpp)
target_link_libraries(px4_style_ekf_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

## px4_gazebo_interface 可执行
add_executable(px4_gazebo_interface src/px4_gazebo_interface.cpp)
target_link_libraries(px4_gazebo_interface
  ${catkin_LIBRARIES}
)

## px4_ekf_simple_node 可执行
add_executable(px4_ekf_simple_node src/px4_ekf_simple.cpp)
add_dependencies(px4_ekf_simple_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(px4_ekf_simple_node
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
)

#################################
## Install executable scripts ##
#################################
catkin_install_python(PROGRAMS
  scripts/republish_odom.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Install ##
#############
install(TARGETS
  px4_style_ekf_node
  px4_gazebo_interface
  px4_ekf_simple_node
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

