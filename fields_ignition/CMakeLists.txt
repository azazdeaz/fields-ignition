cmake_minimum_required(VERSION 3.0.2)
project(fields_ignition)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  tf2
  tf2_ros
)

catkin_package()

catkin_install_python(PROGRAMS scripts/ground_truth.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(pose_tf_broadcaster
  src/pose_tf_broadcaster.cpp
)

target_link_libraries(pose_tf_broadcaster
  ${catkin_LIBRARIES}
)

install(TARGETS pose_tf_broadcaster
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)