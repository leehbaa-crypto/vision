cmake_minimum_required(VERSION 3.0.2)
project(nrs_hand)

# C++ 표준 설정
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(VISP_DIR "/usr/local/lib/cmake/visp")


find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
)

# ViSP 및 RealSense2 라이브러리 찾기
find_package(VISP REQUIRED)
find_package(realsense2 REQUIRED)

# Boost 라이브러리 찾기
find_package(Boost REQUIRED COMPONENTS system thread)

# catkin 패키지 구성
catkin_package()

# include 디렉터리 설정
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${VISP_INCLUDE_DIRS}
  ${realsense2_INCLUDE_DIRS}
)


add_executable(nrs_node_visual_servoing src/nrs_node_visual_servoing.cpp)
target_link_libraries(nrs_node_visual_servoing
  ${catkin_LIBRARIES}
  ${VISP_LIBRARIES}
  Boost::system 
  Boost::thread
)

# Python 스크립트 설치
catkin_install_python(PROGRAMS
  scripts/nrs_node_hand_tracking.py
  scripts/nrs_node_hand_gesture.py
  scripts/nrs_node_record_point.py
  scripts/test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

