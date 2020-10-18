cmake_minimum_required(VERSION 3.5)
project(light_scan_sim)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(ament_cmake)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(OpenCV REQUIRED)

## Generate messages in the 'msg' folder
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Segment.msg"
  "msg/SegmentList.msg"
  "msg/Material.msg"
  "msg/MaterialList.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

add_subdirectory("thirdparty/Box2D")

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${lifecycle_msgs_INCLUDE_DIRS}
  ${rclcpp_lifecycle_INCLUDE_DIRS}
  include
  thirdparty
)

add_executable(light_scan_sim_node 
  src/light_scan_sim_node.cpp
  src/light_scan_sim.cpp
  src/ray_cast.cpp
  src/wall_segment_sim.cpp
)

rosidl_target_interfaces(light_scan_sim_node
      ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

ament_target_dependencies(light_scan_sim_node
  rclcpp
  std_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
)

## Specify libraries to link a library or executable target against
target_link_libraries(light_scan_sim_node
  ${OpenCV_LIBRARIES}
  ${rclcpp_lifecycle_LIBRARIES}
  Box2D_shared
)

install(TARGETS
  light_scan_sim_node
  DESTINATION lib/${PROJECT_NAME}
)

#ament_export_libraries(${PROJECT_NAME})
#ament_export_libraries(Box2D_shared)

ament_package()
