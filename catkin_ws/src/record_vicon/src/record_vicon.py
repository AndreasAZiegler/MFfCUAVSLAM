#!/usr/bin/env python2

# Imports
import rospy
import std_msgs.msg
import geometry_msgs.msg
import csv

# Global variables
list_timestamp_1 = []
list_x_1 = []
list_y_1 = []
list_z_1 = []

list_timestamp_2 = []
list_x_2 = []
list_y_2 = []
list_z_2 = []

## ROS callback function which appends the timestamp and the coordinates of a point for client 0
def position_0_callback(data):
  list_timestamp_1.append(data.header.stamp.secs + 10**(-9)*data.header.stamp.nsecs)
  list_x_1.append(data.transform.translation.x)
  list_y_1.append(data.transform.translation.y)
  list_z_1.append(data.transform.translation.z)

## ROS callback function which appends the timestamp and the coordinates of a point for client 1
def position_1_callback(data):
  list_timestamp_2.append(data.header.stamp.secs + 10**(-9)*data.header.stamp.nsecs)
  list_x_2.append(data.transform.translation.x)
  list_y_2.append(data.transform.translation.y)
  list_z_2.append(data.transform.translation.z)

## ROS callback function which writes all the timestamps and coordinates of the point of both clients into a text file.
def control_callback(data):
  if data.data == True:
    print("Start export")
    with open('export_vicon.csv', 'wb') as csvfile:
      writer = csv.writer(csvfile, delimiter=';')
      for t, x, y, z in zip(list_timestamp_1, list_x_1, list_y_1, list_z_1):
        writer.writerow(['0', "%.8f" % t, x, y, z])
      for t, x, y, z in zip(list_timestamp_2, list_x_2, list_y_2, list_z_2):
        writer.writerow(['1', t, x, y, z])
    print("Export finished")


## Initialize ROS
def initROS():
  rospy.init_node('record_leica', anonymous=True)

  sub_leica_0 = rospy.Subscriber('/retime_messages/position', geometry_msgs.msg.TransformStamped, position_0_callback)
  sub_leica_1 = rospy.Subscriber('/retime_messages/position1', geometry_msgs.msg.TransformStamped, position_1_callback)
  sub_control = rospy.Subscriber('publish_map', std_msgs.msg.Bool, control_callback)

## Main
if __name__ == '__main__':
  try:
    initROS()
    rospy.spin()

  except rospy.ROSInterruptException:
    pass
