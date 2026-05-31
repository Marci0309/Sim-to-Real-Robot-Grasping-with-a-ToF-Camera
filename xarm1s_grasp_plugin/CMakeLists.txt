cmake_minimum_required(VERSION 3.8)
project(xarm1s_grasp_plugin)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)          # Gazebo Classic 11 (gazebo::physics API)
find_package(gazebo_ros REQUIRED)      # gazebo_ros::Node for the ROS 2 side
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(xarm1s_link_attacher SHARED src/link_attacher.cpp)
target_include_directories(xarm1s_link_attacher PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(xarm1s_link_attacher ${GAZEBO_LIBRARIES})
ament_target_dependencies(xarm1s_link_attacher gazebo_ros rclcpp std_msgs)

install(TARGETS xarm1s_link_attacher
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_package()
