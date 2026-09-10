cmake_minimum_required(VERSION 3.0.2)
project(nrs_vision)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_BUILD_TYPE Release)
set(realsense2_DIR /usr/local/lib/cmake/realsense2)
set(Eigen3_DIR /usr/include/eigen3)
set(VTK_DIR "/usr/lib/cmake/vtk-7.1")  # VTK 7.1 설치 경로로 설정
set(PCL_DIR "/usr/local/share/pcl-1.10")

# ROS 필수 패키지 찾기 (roscpp, std_msgs)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pcl_ros
  pcl_conversions
  sensor_msgs
  image_transport
  cv_bridge
  image_geometry
  geometry_msgs
  realsense2
  tf2
  tf2_ros
  tf2_geometry_msgs
  eigen_conversions
  visualization_msgs
)


# 외부 라이브러리 찾기 (PCL, OpenCV, CGAL, VTK, GMP, MPFR)

find_package(VTK REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CGAL REQUIRED)



## 서비스 파일 추가
add_service_files(
  FILES
  Command.srv
)


###################################
## catkin 패키지 설정 ##
###################################
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs image_transport cv_bridge image_geometry
  DEPENDS VTK PCL OpenCV CGAL
)
###################################
## include 디렉토리 설정 ##
###################################
include_directories(
  include
  include
  /usr/local/include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${VTK_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CGAL_INCLUDE_DIRS}
  ${MPFR_INCLUDE_DIR}
  ${realsense2_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS} ${OpenCV_LIBRARY_DIRS})

###################################
## 빌드 설정 ##
###################################

############################################################################################
link_libraries(${REALSENSE2_FOUND}
${catkin_LIBRARIES}
${PCL_LIBRARIES} 
${OpenCV_LIBS} 
${image_transport_LIBRARIES} 
${cv_bridge_LIBRARIES} 
${image_geometry_LIBRARIES}
${CGAL_LIBRARIES}
${YAML_CPP_LIBRARIES})





add_executable(nrs_node_capture 
src/nrs_node_capture.cpp  


src/nrs_pointcloud.cpp
src/nrs_mesh.cpp
src/nrs_callback.cpp
src/nrs_io.cpp
src/nrs_math.cpp)




add_executable(nrs_node_registration
src/nrs_node_registration.cpp


src/nrs_pointcloud.cpp
src/nrs_mesh.cpp

src/nrs_callback.cpp
src/nrs_io.cpp
src/nrs_math.cpp)




add_executable(nrs_node_reconstruction
src/nrs_node_reconstruction.cpp


src/nrs_pointcloud.cpp
src/nrs_mesh.cpp

src/nrs_callback.cpp
src/nrs_io.cpp
src/nrs_math.cpp)

