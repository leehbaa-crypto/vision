cmake_minimum_required(VERSION 3.0.2)
project(nrs_mcp)

## 빌드 옵션
add_definitions(-Wall -Wextra)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  message_generation
  moveit_ros_planning_interface
  tf2_eigen
  pcl_ros
  pcl_conversions
  visualization_msgs
)

## 서비스 파일
add_service_files(FILES
  GoToPose.srv
  GetTcpPose.srv
)

generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    roscpp
    std_msgs
    geometry_msgs
    tf2_ros
    tf2_geometry_msgs
    moveit_ros_planning_interface
)

include_directories(${catkin_INCLUDE_DIRS})

## 실행 파일
add_executable(moveit_go_to_pose_server src/moveit_go_to_pose_server.cpp)
add_dependencies(moveit_go_to_pose_server
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(moveit_go_to_pose_server ${catkin_LIBRARIES})

add_executable(get_tcp_pose_server src/get_tcp_pose_server.cpp)
add_dependencies(get_tcp_pose_server
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(get_tcp_pose_server ${catkin_LIBRARIES})

add_executable(pcd_publisher src/pcd_publisher.cpp)
target_link_libraries(pcd_publisher ${catkin_LIBRARIES})

add_executable(clicked_points_logger src/clicked_points_logger.cpp)
target_link_libraries(clicked_points_logger ${catkin_LIBRARIES})

add_executable(central_supervisor_node src/central_supervisor.cpp)
target_link_libraries(central_supervisor_node ${catkin_LIBRARIES})

## 설치 규칙 (옵션이지만 추천)
install(TARGETS moveit_go_to_pose_server get_tcp_pose_server
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

install(FILES
  srv/GoToPose.srv
  srv/GetTcpPose.srv
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/srv
)
