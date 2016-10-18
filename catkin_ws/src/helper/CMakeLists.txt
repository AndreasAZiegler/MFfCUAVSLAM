cmake_minimum_required(VERSION 2.8.3)
project(helper)

#set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-std=c++11 -pthread ${CMAKE_CXX_FLAGS}")

#ROS_BUILD_TYPE: Set the build type. Options are (default: RelWithDebInfo):
#Debug : w/ debug symbols, w/o optimization
#Release : w/o debug symbols, w/ optimization
#RelWithDebInfo : w/ debug symbols, w/ optimization
#RelWithAsserts : w/o debug symbols, w/ optimization, w/ assertions (i.e., w/o -DNDEBUG). New in ros 1.1.
#MinSizeRel : w/o debug symbols, w/ optimization, stripped binaries
set(CMAKE_BUILD_TYPE RelWithDebInfo)
#set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS roscpp cv_bridge tf pcl_ros)
find_package(cmake_modules REQUIRED)
find_package(OpenCV REQUIRED)
#find_package(Eigen REQUIRED 3.0)
#include_directories(${EIGEN_INCLUDE_DIRS})
#add_definitions(${EIGEN_DEFINITIONS})
#find_package(PCL 1.3 REQUIRED)



catkin_package(
   INCLUDE_DIRS include
   #LIBRARIES opencv_features2d
   #CATKIN_DEPENDS mcpslam_msgs
)

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  #${PCL_INCLUDE_DIRS}
  #${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

set(EXTERNAL_LIBRARIES
  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
#  ${EIGEN3_LIBS}
)

ADD_SUBDIRECTORY( include/helper )

add_library(estd
  include/helper/estd.h
  src/estd.cpp
)
target_link_libraries(estd ${EXTERNAL_LIBRARIES})

add_library(libhelper
  include/helper/helperfuncs.h
  src/helperfuncs.cpp
)
target_link_libraries(estd ${EXTERNAL_LIBRARIES})

add_executable(republisher_node
  include/helper/republisher.h
  src/republisher.cpp
  src/republisher_node.cpp
)
#add_dependencies(republisher_node mcpslam_msgs mcpslam_msgs_generate_messages_cpp)
target_link_libraries(republisher_node ${EXTERNAL_LIBRARIES})

add_executable(mcptest_node
  src/test.cpp
)
#add_dependencies(mcptest_node mcpslam_msgs mcpslam_msgs_generate_messages_cpp)
target_link_libraries(mcptest_node ${EXTERNAL_LIBRARIES})

add_executable(imagesampler_node
  src/imagesampler_node.cpp

  include/helper/imagesampler.h
  src/imagesampler.cpp
)
target_link_libraries(imagesampler_node ${EXTERNAL_LIBRARIES})

add_executable(macslam_viewer_node
  src/viewer_node.cpp

  include/helper/viewer.h
  src/viewer.cpp
)
target_link_libraries(macslam_viewer_node ${EXTERNAL_LIBRARIES})
