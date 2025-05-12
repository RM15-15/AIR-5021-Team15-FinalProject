cmake_minimum_required(VERSION 3.0.2)
project(sgr_waypoint_runner)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

install(
  PROGRAMS
    src/sgr_ctrl.py
    src/txt_waypoint_executor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
