cmake_minimum_required(VERSION 3.8)
project(lidar_odometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(message_filters REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fast_gicp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(lidar_odometry_node
  src/lidar_odometry_node.cpp
  src/sdk_pose_buffer.cpp
  src/imu_buffer.cpp
  src/deskew.cpp
  src/icp_odometry.cpp
  src/ground_removal.cpp
)

target_include_directories(lidar_odometry_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(lidar_odometry_node
  rclcpp
  sensor_msgs
  nav_msgs
  pcl_ros
  pcl_conversions
  message_filters
  Eigen3
  fast_gicp
  tf2_ros
  geometry_msgs
)
target_link_libraries(lidar_odometry_node Eigen3::Eigen)

install(TARGETS lidar_odometry_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()