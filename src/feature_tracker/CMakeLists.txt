cmake_minimum_required(VERSION 3.8)
project(feature_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(OpenCV REQUIRED)

add_executable(feature_tracker_node src/feature_tracker_node.cpp)

ament_target_dependencies(feature_tracker_node
  rclcpp
  sensor_msgs
  cv_bridge
)

target_link_libraries(feature_tracker_node
  ${OpenCV_LIBS}
)

install(
  TARGETS feature_tracker_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
