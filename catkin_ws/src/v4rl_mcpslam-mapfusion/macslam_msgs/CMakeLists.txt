cmake_minimum_required(VERSION 2.8.3)
project(macslam_msgs)

find_package(catkin REQUIRED COMPONENTS 
  message_generation
  message_runtime
  geometry_msgs 
  roscpp
  sensor_msgs
  std_msgs
)

add_message_files(
  FILES
  macMap.msg
  macKeyFrame.msg
  macMapPoint.msg
  machCvKeyPoint.msg
  machUIntVec.msg
  machGridEntries.msg
  machGridCols.msg
  mach2DGrid.msg
  machDescriptor.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES multi_octo_msgs
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
# braucht man das?? evtl wieder entfernen...
