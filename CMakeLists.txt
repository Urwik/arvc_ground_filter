cmake_minimum_required(VERSION 3.0.2)
project(arvc_ground_filter)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)


# find_package(PCL 1.3 REQUIRED COMPONENTS common io filters visualization)
find_package(PCL 1.3 REQUIRED)
message(STATUS "PCL version: ${PCL_VERSION}")

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(include  ${PCL_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${Eigen3_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS} ${Eigen3_DEFINITIONS})



include_directories(arvc_utils/include)



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES arvc_ground_filter
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rrss_grnd_filter src/rrss_grnd_filter.cpp)
target_link_libraries(rrss_grnd_filter ${PCL_LIBRARIES})

