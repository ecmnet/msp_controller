cmake_minimum_required(VERSION 3.5)
project(msp_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-address-of-packed-member -O2 )
endif()

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include_directories(include)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(msp_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(glog REQUIRED)

add_library(msp_transformer SHARED src/lib/tf_transformer.cc)
target_include_directories(msp_transformer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>  # Use 'include' when installed
)

ament_target_dependencies(msp_transformer rclcpp tf2 tf2_ros px4_msgs msp_msgs geometry_msgs Eigen3 )

add_executable(msp_controller src/msp_controller.cpp 
                              src/msp_mavlink_dispatcher.cpp
                              src/msp_time_sync.cpp
                              src/msp_status_manager.cpp
                              src/lib/tf_transformer.cc
                              )

ament_target_dependencies(msp_controller rclcpp px4_msgs msp_msgs geometry_msgs Eigen3 tf2 tf2_ros )

target_link_libraries(msp_controller pthread Eigen3::Eigen glog )

ament_export_targets(msp_transformer_export HAS_LIBRARY_TARGET)
ament_export_dependencies(ament_cmake rclcpp rosidl_default_runtime px4_msgs  tf2 tf2_ros)
ament_export_include_directories(include)

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

install(TARGETS
  msp_controller 
  DESTINATION lib/${PROJECT_NAME})

  install(TARGETS
  msp_transformer
  EXPORT msp_transformer_export 
  DESTINATION lib/msp_transformer)



ament_package()
