cmake_minimum_required(VERSION 3.0.2)
project(iphone_slam)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
  # pcl library
  pcl_ros
  pcl_conversions
)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
catkin_python_setup()

include_directories(
  include  
  ${Eigen3_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# link directories
link_directories(
  include
  ${PCL_LIBRARY_DIRS}  
)

install(PROGRAMS
  scripts/iphoneros_bak.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/pub_iphone_trans.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/pub_odom.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Declare a catkin package
catkin_package()
