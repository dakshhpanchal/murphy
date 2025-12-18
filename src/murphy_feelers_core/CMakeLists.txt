cmake_minimum_required(VERSION 3.8)
project(murphy_feelers_core)

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)

add_library(feelers_core
  src/feeler.cpp
)

target_include_directories(feelers_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(feelers_core ${OpenCV_LIBS})
target_include_directories(feelers_core SYSTEM PUBLIC ${OpenCV_INCLUDE_DIRS})
install(
  TARGETS feelers_core
  EXPORT export_feelers_core
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_feelers_core)
ament_export_include_directories(include)
ament_export_dependencies(OpenCV)

ament_package()
