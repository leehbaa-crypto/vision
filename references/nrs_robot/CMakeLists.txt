cmake_minimum_required(VERSION 3.0.2)
project(nrs_robot)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_srvs
  std_msgs
  moveit_ros_planning_interface
)


catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_srvs
    moveit_ros_planning_interface
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


# add_executable(nrs_node_simulation
# src/nrs_node_simulation.cpp
# )
# target_link_libraries(nrs_node_simulation
#   ${catkin_LIBRARIES}
# )


add_executable(nrs_node_ur10_execution src/nrs_node_ur10_execution.cpp)
target_link_libraries(nrs_node_ur10_execution ${catkin_LIBRARIES})

# # Python 스크립트 설치
# catkin_install_python(PROGRAMS
#   scripts/nrs_node_stream_camera.py
#   scripts/nrs_node_discrete_teaching.py
#   scripts/nrs_node_continuous_teaching.py
#   scripts/test.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

