cmake_minimum_required(VERSION 3.0.2)
project(vehicle_params)

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
)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs std_msgs nav_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
