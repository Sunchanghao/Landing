cmake_minimum_required(VERSION 2.8.3)
project(px4_control)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation  
  roscpp
  geometry_msgs
  sensor_msgs
  mavros
  nav_msgs
  std_msgs
  std_srvs
  tf2_ros
  tf2_eigen
  mavros_msgs
  actionlib
  trajectory_msgs
  pluginlib
)
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES vision_to_mavros
#  CATKIN_DEPENDS roscpp rospy tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)



add_executable(landing_quadrotor_node src/landing_quadrotor.cpp)
add_executable(tracking_quadrotor_node src/tracking_quadrotor.cpp)

add_executable(tracking_quadrotor_yolo_node src/tracking_quadrotor_yolo.cpp)
add_executable(tracking_quadrotor_pid_node src/tracking_quadrotor_pid.cpp)
add_executable(ar_pose src/ar_pose.cpp)
add_executable(px4pos_gazebo_node src/px4pos_gazebo.cpp)
add_executable(picking_gazebo_pos_node src/picking_gazebo_pos.cpp)
## Specify libraries to link a library or executable target against



target_link_libraries(landing_quadrotor_node
  ${catkin_LIBRARIES}
)
target_link_libraries(tracking_quadrotor_node
  ${catkin_LIBRARIES}
)
target_link_libraries(tracking_quadrotor_yolo_node
  ${catkin_LIBRARIES}
)
target_link_libraries(tracking_quadrotor_pid_node
  ${catkin_LIBRARIES}
)

target_link_libraries(ar_pose
  ${catkin_LIBRARIES}
)
target_link_libraries(px4pos_gazebo_node
  ${catkin_LIBRARIES}
)
target_link_libraries(picking_gazebo_pos_node
  ${catkin_LIBRARIES}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
## Mark executables and/or libraries for installation
install(TARGETS  position_to_mavros_node
  set_mode_node
  collision_avoidance_vfh_node
  avoidance_car_vfh_node
  collision_avoidance_matlabvfh_node
  circular_node circular_car_node 
  ros_nav_quadrotor_node 
  mission_car_offboard_node
  landing_quadrotor_node
  tracking_quadrotor_node
 tracking_quadrotor_yolo_node
  auto_2d_nav_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
