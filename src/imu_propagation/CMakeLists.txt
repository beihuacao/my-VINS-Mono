cmake_minimum_required(VERSION 3.8)
project(imu_propagation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(imu_propagator
  src/imu_propagator.cpp
)

target_include_directories(imu_propagator
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(imu_propagator
  Eigen3::Eigen
)

install(
  TARGETS imu_propagator
  EXPORT imu_propagator_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_targets(
  imu_propagator_targets
  HAS_LIBRARY_TARGET
)

ament_export_dependencies(Eigen3)

add_executable(imu_propagation_node
  src/imu_propagation_node.cpp
)

ament_target_dependencies(imu_propagation_node
  rclcpp
  sensor_msgs
)

install(
  TARGETS imu_propagation_node
  DESTINATION lib/${PROJECT_NAME}
)

target_link_libraries(imu_propagation_node
  imu_propagator
)


if(BUILD_TESTING)
find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(imu_propagator_test
  test/imu_propagator_test.cpp
)

target_link_libraries(imu_propagator_test
  imu_propagator
)
endif()


ament_package()
