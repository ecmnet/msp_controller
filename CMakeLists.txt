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

#find_package(Boost REQUIRED thread)
#find_package(eigen3_cmake_module REQUIRED)
#find_package(Eigen3 REQUIRED)


add_executable(msp_controller src/msp_controller.cpp 
                              src/msp_mavlink_dispatcher.cpp
                              src/msp_time_sync.cpp
                              src/msp_status_manager.cpp
                              )
ament_target_dependencies(msp_controller rclcpp px4_msgs msp_msgs geometry_msgs )

target_link_libraries(msp_controller pthread )


ament_export_dependencies(ament_cmake rclcpp rosidl_default_runtime px4_msgs)
ament_export_include_directories(include)


install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

install(TARGETS
  msp_controller 
  DESTINATION lib/${PROJECT_NAME})

ament_package()
