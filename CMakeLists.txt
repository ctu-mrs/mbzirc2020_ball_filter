cmake_minimum_required(VERSION 2.8.3)
project(ball_filter)

set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -O0 -g -DEIGEN_INITIALIZE_MATRICES_BY_NAN")
set(CMAKE_CXX_FLAGS_DEBUG "-std=c++17 -Wall -O0 -g -DEIGEN_INITIALIZE_MATRICES_BY_NAN")
set(CMAKE_CXX_FLAGS_RELEASE "-std=c++17 -Wall -O3 -g")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-std=c++17 -Wall -O3 -g")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure
  std_msgs
  geometry_msgs
  nav_msgs
  mrs_lib
  pcl_ros
  message_generation
  uav_detect
  pcl_ros
  )

# include Eigen3
find_package(Eigen3 REQUIRED)

find_package(PCL REQUIRED COMPONENTS)

generate_dynamic_reconfigure_options(
  config/FilterParams.cfg
  )

#Generate messages in the 'msg' folder
add_message_files(
   FILES
   BallLocation.msg
   Plane.msg
   LKFState.msg
   FilterState.msg
   BallPrediction.msg
   PlaneStamped.msg
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
   geometry_msgs
   nav_msgs
 )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES BallFilter
  CATKIN_DEPENDS
  #  DEPENDS system_lib
  )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  )

## Add link directories
link_directories(
  ${PCL_LIBRARY_DIRS}
)

# This causes adds the -march=native gcc flag that prevents ditributed compilation
add_definitions(
  ${PCL_DEFINITIONS}
  )

add_library(BallFilter
  src/plane_rheiv.cpp
  src/vel_lkf.cpp
  src/BallFilter.cpp
  )
add_dependencies(BallFilter
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})
target_link_libraries(BallFilter
  ${catkin_LIBRARIES}
  )
