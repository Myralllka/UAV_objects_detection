cmake_minimum_required(VERSION 3.1.2)
project(mrs_detection)

set(FILESNAME DetectionNode)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Override CXX flags inherited from workspace, if precompiled PCL binaries from debian repos are used
if (DEFINED ENV{PCL_CROSS_COMPILATION})
  set(PCL_CROSS_COMPILATION $ENV{PCL_CROSS_COMPILATION})
else()
  set(PCL_CROSS_COMPILATION "false")
endif()
if(${PCL_CROSS_COMPILATION} STREQUAL "false")
  message("Using precompiled PCL binaries from debian repos. Overriding catkin workspace CMAKE_CXX_FLAGS.")
  set(CMAKE_CXX_FLAGS "-std=c++17")
else()
  message("Using custom-built PCL binaries. Inheriting all CMAKE_CXX_FLAGS from catkin workspace.")
endif()

add_compile_options(-Wall -Wextra -pedantic -Werror -Werror=vla)
add_compile_options("-DEIGEN_INITIALIZE_MATRICES_BY_NAN")
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  nav_msgs
  mrs_lib
  tf2_eigen
  tf2_ros
  tf2_eigen
  geometry_msgs
  swarm_gazebo_resources
)

add_message_files(
  FILES
  Metadata.msg
  MetadataArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

find_package(OpenCV REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  CATKIN_DEPENDS roscpp mrs_lib message_runtime std_msgs
  DEPENDS Eigen
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(${FILESNAME} src/${FILESNAME}.cpp)

add_dependencies(${FILESNAME} ${${FILESNAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${FILESNAME} ${catkin_LIBRARIES} ${Eigen_LIBRARIES})
