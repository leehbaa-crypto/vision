cmake_minimum_required(VERSION 3.0.2)
project(nrs_solution)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_srvs
  std_msgs
  message_generation

)
add_message_files(
  FILES
  HandGesture.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs
EXPORTED_TARGETS ${PROJECT_NAME}_generate_messages_cpp)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(nrs_node_central_FSM src/nrs_node_central_FSM.cpp)
target_link_libraries(nrs_node_central_FSM ${catkin_LIBRARIES})
add_dependencies(nrs_node_central_FSM ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


# Python 스크립트 설치
catkin_install_python(PROGRAMS
  scripts/nrs_node_stream_camera.py
  scripts/nrs_node_discrete_teaching.py
  scripts/nrs_node_continuous_teaching.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

