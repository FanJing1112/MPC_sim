cmake_minimum_required(VERSION 3.14)
project(simulation)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")


find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)

find_package(catkin REQUIRED COMPONENTS
#    roscpp
#    std_msgs
#    geometry_msgs
##    tf
#    nav_msgs
#    sensor_msgs
##    cv_bridge
#    mavros
#    mavros_extras
#    mavros_msgs
#    tf2_ros
#    pcl_conversions
#    visualization_msgs
#    pcl_ros
    roscpp
    nav_msgs
    geometry_msgs
    sensor_msgs
    std_msgs
    pcl_conversions
    pcl_ros
)

#find_package(OpenCV REQUIRED)

# message( "OpenCV_VERSION: ${OpenCV_VERSION}")


#include_directories(${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})

#set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
#find_package(Eigen3)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OsqpEigen_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIR}
)

catkin_package(
    CATKIN_DEPENDS
)


add_executable(simulation_1 src/simulation.cpp)
target_link_libraries(simulation_1 ${catkin_LIBRARIES} ${EIGEN3_LIBS} OsqpEigen::OsqpEigen) 

#add_executable(simulation_2 src/main2.cpp)
#target_link_libraries(simulation_2 ${catkin_LIBRARIES} ${EIGEN3_LIBS} OsqpEigen::OsqpEigen)
