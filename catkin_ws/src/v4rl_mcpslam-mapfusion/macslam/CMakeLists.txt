cmake_minimum_required(VERSION 2.8.3)
project(macslam)

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
#set(CMAKE_BUILD_TYPE Release)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS roscpp cv_bridge tf macslam_msgs helper pcl_ros tf_conversions)
find_package(cmake_modules REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen REQUIRED 3.0)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})
find_package(PCL 1.3 REQUIRED)



catkin_package(
   INCLUDE_DIRS include
   #LIBRARIES opencv_features2d
   CATKIN_DEPENDS macslam_msgs
   LIBRARIES mac_slam_lib
)

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

set(EXTERNAL_LIBRARIES
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${EIGEN3_LIBS}
)

ADD_SUBDIRECTORY( include/macslam )

add_library(mac_slam_lib
#cpp files
  src/ClientHandler.cpp
  src/Communicator.cpp
  src/Converter.cc
  src/Database.cpp
  src/Frame.cpp
  src/Initializer.cpp
  src/KeyFrame.cpp
  src/LoopFinder.cpp
  src/Map.cpp
  src/MapMatcher.cpp
  src/MapMerger.cpp
  src/Mapping.cpp
  src/MapPoint.cpp
  src/Optimizer.cpp
  src/ORBextractor.cpp
  src/ORBmatcher.cpp
  src/PnPSolver.cpp
  src/Sim3Solver.cpp
  src/Tracking.cpp
  src/Viewer.cpp

#header
  include/macslam/CentralControl.h
  include/macslam/ClientHandler.h
  include/macslam/Communicator.h
  include/macslam/Converter.h
  include/macslam/Database.h
  include/macslam/Datatypes.h
  include/macslam/Frame.h
  include/macslam/Initializer.h
  include/macslam/KeyFrame.h
  include/macslam/LoopFinder.h
  include/macslam/Map.h
  include/macslam/MapMatcher.h
  include/macslam/MapMerger.h
  include/macslam/Mapping.h
  include/macslam/MapPoint.h
  include/macslam/Optimizer.h
  include/macslam/ORBextractor.h
  include/macslam/ORBmatcher.h
  include/macslam/ORBVocabulary.h
  include/macslam/PnPSolver.h
  include/macslam/Sim3Solver.h
  include/macslam/Tracking.h
  include/macslam/Viewer.h

)

add_dependencies(mac_slam_lib macslam_msgs macslam_msgs_generate_messages_cpp)
target_link_libraries(mac_slam_lib ${EXTERNAL_LIBRARIES} ${PROJECT_SOURCE_DIR}/thirdparty/g2o/lib/libg2o.so ${PROJECT_SOURCE_DIR}/thirdparty/DBoW2/lib/libDBoW2.so)

add_executable(macClientNode
#node
  src/client/ClientNode.cpp

#cpp files
  src/client/ClientSystem.cpp

#header
  include/macslam/client/ClientSystem.h
)

add_dependencies(macClientNode mac_slam_lib)
target_link_libraries(macClientNode ${EXTERNAL_LIBRARIES} mac_slam_lib)

add_executable(macServerNode
#node
  src/server/ServerNode.cpp

#cpp files
  src/server/ServerSystem.cpp

#header
  include/macslam/server/ServerSystem.h
)

add_dependencies(macServerNode mac_slam_lib)
target_link_libraries(macServerNode ${EXTERNAL_LIBRARIES} mac_slam_lib)
