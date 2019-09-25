cmake_minimum_required(VERSION 2.8.3)
project(balloon_filter)

set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -O0 -g -fvar-tracking -fvar-tracking-assignments -DEIGEN_INITIALIZE_MATRICES_BY_NAN")
set(CMAKE_CXX_FLAGS_DEBUG "-std=c++17 -Wall -O0 -g -fvar-tracking -fvar-tracking-assignments -DEIGEN_INITIALIZE_MATRICES_BY_NAN")
set(CMAKE_CXX_FLAGS_RELEASE "-std=c++17 -Wall -O0 -g -fvar-tracking -fvar-tracking-assignments -DEIGEN_INITIALIZE_MATRICES_BY_NAN")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure
  mrs_msgs
  mrs_lib
  pcl_ros
  message_generation
  object_detect
  )

# include Eigen3
find_package(Eigen3 REQUIRED)

generate_dynamic_reconfigure_options(
  config/FilterParams.cfg
  )

#Generate services in the 'srv' folder
add_service_files(
   FILES
   ResetChosen.srv
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES BalloonFilter
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
  )

## Declare a C++ executable
add_library(BalloonFilter
  src/eight_ukf.cpp
  src/eight_rheiv.cpp
  src/BalloonFilter.cpp
  )
add_dependencies(BalloonFilter
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})
## Specify libraries to link a library or executable target against
target_link_libraries(BalloonFilter
  ${catkin_LIBRARIES}
  )

## Declare a C++ executable
add_executable(eight_ukf
  src/eight_ukf.cpp
  )
add_dependencies(eight_ukf
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})
## Specify libraries to link a library or executable target against
target_link_libraries(eight_ukf
  ${catkin_LIBRARIES}
  )
target_compile_options(eight_ukf PRIVATE "-DCOMPILE_EIGHT_UKF_TEST")

## Declare a C++ executable
add_executable(eight_rheiv
  src/eight_rheiv.cpp
  )
add_dependencies(eight_rheiv
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})
## Specify libraries to link a library or executable target against
target_link_libraries(eight_rheiv
  ${catkin_LIBRARIES}
  )
target_compile_options(eight_rheiv PRIVATE "-DCOMPILE_EIGHT_RHEIV_TEST")
