cmake_minimum_required(VERSION 3.8)
project(mapping)

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

include_directories(include ${cilantro_INCLUDE_DIRS})

add_executable(map_data_save_node src/map_data_save_node.cpp)
ament_target_dependencies(map_data_save_node rclcpp 
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
  map_data_save_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
