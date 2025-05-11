cmake_minimum_required(VERSION 3.0.2)
project(cbf_module)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  octomap_msgs
  mrs_msgs
  visualization_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(cbf_filter_node src/cbf_filter.cpp)
target_link_libraries(cbf_filter_node 
    ${catkin_LIBRARIES}
    octomap
)
