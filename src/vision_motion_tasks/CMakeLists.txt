cmake_minimum_required(VERSION 3.0.2)
project(vision_motion_tasks)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/gaze_controller.py
  scripts/person_follow_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
