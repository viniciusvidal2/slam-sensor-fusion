cmake_minimum_required(VERSION 3.8)
project(localization)

# Enable C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(message_filters REQUIRED)

# find PCL
find_package(PCL REQUIRED)

include_directories(include)

add_executable(localization_node src/main.cpp 
              src/localization_node.cpp
              src/global_map_frames_manager.cpp
              src/icp_point_to_point.cpp
              src/stochastic_filter.cpp
              src/brute_force_alignment.cpp)
              
ament_target_dependencies(localization_node rclcpp 
rclcpp_components 
geometry_msgs 
sensor_msgs 
std_msgs
nav_msgs 
pcl_ros 
pcl_conversions 
message_filters
)

install(TARGETS
  localization_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
