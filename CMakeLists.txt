cmake_minimum_required(VERSION 3.10)
project(cbf_filter)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

############################ 1. catkin deps ###################################
find_package(catkin REQUIRED COMPONENTS
    roscpp
    nav_msgs
    visualization_msgs
    mrs_msgs
    voxblox_ros
)

############################ 2. plain-CMake deps ##############################
find_package(Eigen3      REQUIRED)

############################ 3. catkin package ################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES    cbf_filter
  CATKIN_DEPENDS roscpp nav_msgs mrs_msgs visualization_msgs voxblox_ros
)

############################ 4. include paths #################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

############################ 5. executable ####################################
add_executable(cbf_filter_node  src/cbf_filter.cpp)

# Directly link the known library path
target_link_libraries(cbf_filter_node
  ${catkin_LIBRARIES}
)

############################ 6. install (optional) ############################
install(TARGETS cbf_filter_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
