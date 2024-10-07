cmake_minimum_required(VERSION 3.0.2)
project(mapping)

# Enable C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find catkin dependencies
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

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mapping
  CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs std_msgs nav_msgs pcl_ros pcl_conversions message_filters
  DEPENDS PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(map_data_save_node src/main.cpp 
                                  src/map_data_save_node.cpp)

target_link_libraries(map_data_save_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

install(TARGETS
  map_data_save_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
