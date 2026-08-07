cmake_minimum_required(VERSION 3.15)
project(isaaclab_height_scan_builder)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(cppzmq REQUIRED)
set(protobuf_MODULE_COMPATIBLE ON)
find_package(Protobuf CONFIG REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PCL REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(visualization_msgs REQUIRED)

protobuf_generate_cpp(
  PROTO_SRCS PROTO_HDRS
  proto/point_scan_msg.proto
  proto/height_scan_msg.proto
)

add_executable(height_scan_builder_node
  src/main.cpp
  src/height_scan_builder.cpp
  ${PROTO_SRCS}
)

target_include_directories(height_scan_builder_node PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(height_scan_builder_node
  ${PCL_LIBRARIES}
  cppzmq
  protobuf::libprotobuf
)

ament_target_dependencies(height_scan_builder_node
  pcl_conversions
  rclcpp
  sensor_msgs
  tf2_ros
  visualization_msgs
)

add_executable(point_cloud_manager_node
  src/point_cloud_manager.cpp
)

add_executable(imu_world_frame_publisher_node
  src/imu_world_frame_publisher.cpp
)

add_executable(lidar_extrinsic_calibrator_node
  src/lidar_extrinsic_calibrator.cpp
)

target_include_directories(point_cloud_manager_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${PCL_INCLUDE_DIRS}
)

target_include_directories(lidar_extrinsic_calibrator_node PRIVATE
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(point_cloud_manager_node
  ${PCL_LIBRARIES}
)

target_link_libraries(lidar_extrinsic_calibrator_node
  ${PCL_LIBRARIES}
)

target_link_libraries(imu_world_frame_publisher_node
  ${PCL_LIBRARIES}
)

ament_target_dependencies(point_cloud_manager_node
  pcl_conversions
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  message_filters
)

ament_target_dependencies(lidar_extrinsic_calibrator_node
  message_filters
  pcl_conversions
  rclcpp
  sensor_msgs
)

ament_target_dependencies(imu_world_frame_publisher_node
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS height_scan_builder_node point_cloud_manager_node imu_world_frame_publisher_node lidar_extrinsic_calibrator_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY proto
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY params
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
