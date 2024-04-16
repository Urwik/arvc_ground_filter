cmake_minimum_required(VERSION 3.0.2)
project(arvc_ground_filter)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)


find_package(PCL 1.10 REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Yaml-cpp REQUIRED)
find_package(Threads REQUIRED)

include_directories(include  ${PCL_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${Eigen3_LIBRARY_DIRS} ${YAML_CPP_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS} ${Eigen3_DEFINITIONS} ${YAML_CPP_DEFINITIONS})



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
  ${catkin_INCLUDE_DIRS}
)



add_library(arvc_ground_filter src/ground_filter.cpp)
target_link_libraries(arvc_ground_filter ${PCL_LIBRARIES} ${Eigen3_LIBRARIES})

add_executable(ground_filter_node src/node.cpp)
target_link_libraries(ground_filter_node arvc_ground_filter ${CATKIN_LIBRARIES} ${PCL_LIBRARIES} ${Eigen3_LIBRARIES} ${YAML_CPP_LIBRARIES})

add_executable(experiment_queue_node src/experiment_queue.cpp)
target_link_libraries(experiment_queue_node arvc_ground_filter ${CATKIN_LIBRARIES} ${PCL_LIBRARIES} ${Eigen3_LIBRARIES} ${YAML_CPP_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})