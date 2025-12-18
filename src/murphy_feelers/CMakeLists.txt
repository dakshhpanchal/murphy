cmake_minimum_required(VERSION 3.8)
project(murphy_feelers)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(murphy_feelers_core REQUIRED)

add_executable(feeler_node src/feeler_node.cpp)

target_link_libraries(feeler_node
  murphy_feelers_core::feelers_core
  ${OpenCV_LIBS}
)

ament_target_dependencies(feeler_node
  rclcpp
  sensor_msgs
  cv_bridge
)

install(TARGETS feeler_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
