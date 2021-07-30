cmake_minimum_required(VERSION 3.0.2)
project(localizer)

find_package(OpenCV 4.0.0 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  geometry_msgs
  image_transport
  tf
  aruco
  aruco_msgs
  aruco_ros
)

add_compile_options(-std=c++11)

catkin_package(
INCLUDE_DIRS include
CATKIN_DEPENDS aruco
)

include_directories(
  include)

include_directories(
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(localizer src/localizer.cpp)
add_dependencies(localizer ${catkin_EXPORTED_TARGETS})
target_link_libraries(localizer ${catkin_LIBRARIES})

add_executable(localizer_adj src/localizer_ref_adjusted.cpp)
add_dependencies(localizer_adj ${catkin_EXPORTED_TARGETS})
target_link_libraries(localizer_adj ${catkin_LIBRARIES})

