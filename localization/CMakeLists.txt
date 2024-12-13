cmake_minimum_required(VERSION 3.0.2)
project(localization)

# Enable C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  std_msgs
  nav_msgs
  pcl_ros
  pcl_conversions
  message_filters
)

# find PCL
find_package(PCL REQUIRED)

# Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES localization
  CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs std_msgs nav_msgs pcl_ros pcl_conversions message_filters
  DEPENDS PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(localization_node src/localization_main.cpp 
              src/localization_node.cpp
              src/global_map_frames_manager.cpp
              src/icp_point_to_point.cpp)              
target_link_libraries(localization_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
install(TARGETS localization_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(obstacle_generator_node src/obstacle_point_cloud_generator_main.cpp 
              src/obstacle_point_cloud_generator_node.cpp
              src/global_map_frames_manager.cpp)              
target_link_libraries(obstacle_generator_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
install(TARGETS obstacle_generator_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
