#!/usr/bin/env python2

# Imports
import rospy
import sensor_msgs.msg
import geometry_msgs.msg

# Global variables
pub_uav_cam_0 = rospy.Publisher('/retime_messages/image_raw', sensor_msgs.msg.Image, queue_size=1)
pub_uav_cam_1 = rospy.Publisher('/retime_messages/image_raw1', sensor_msgs.msg.Image, queue_size=1)
#pub_leica_0 = rospy.Publisher('/retime_messages/position', geometry_msgs.msg.PointStamped, queue_size=1)
#pub_leica_1 = rospy.Publisher('/retime_messages/position1', geometry_msgs.msg.PointStamped, queue_size=1)
pub_vicon_0 = rospy.Publisher('/retime_messages/position', geometry_msgs.msg.TransformStamped, queue_size=1)
pub_vicon_1 = rospy.Publisher('/retime_messages/position1', geometry_msgs.msg.TransformStamped, queue_size=1)

## ROS callback function which retimes the grayscale image of client 0
def uav_cam_0_callback(data):
  data.header.stamp = rospy.Time.now()
  #print "time.now(): " + str(rospy.Time.now())
  pub_uav_cam_0.publish(data)

## ROS callback function which retimes the grayscale image of client 1
def uav_cam_1_callback(data):
  data.header.stamp = rospy.Time.now()
  #print "time.now(): " + str(rospy.Time.now())
  pub_uav_cam_1.publish(data)

## ROS callback function which retimes the ground truth position from the leica for client 0
def leica_0_callback(data):
  data.header.stamp = rospy.Time.now()
  pub_leica_0.publish(data)

## ROS callback function which retimes the ground truth position from the leica for client 1
def leica_1_callback(data):
  data.header.stamp = rospy.Time.now()
  pub_leica_1.publish(data)

## ROS callback function which retimes the ground truth position from the vicon for client 0
def vicon_0_callback(data):
  data.header.stamp = rospy.Time.now()
  pub_vicon_0.publish(data)

## ROS callback function which retimes the ground truth position from the vicon for client 1
def vicon_1_callback(data):
  data.header.stamp = rospy.Time.now()
  pub_vicon_1.publish(data)

## Initialize ROS
def initROS():
  rospy.init_node('retime_messages', anonymous=True)

  sub_uav_cam_0 = rospy.Subscriber('/cam0/image_raw', sensor_msgs.msg.Image, uav_cam_0_callback)
  sub_uav_cam_1 = rospy.Subscriber('/cam0/image_raw1', sensor_msgs.msg.Image, uav_cam_1_callback)
  #sub_leica_0 = rospy.Subscriber('/leica/position', geometry_msgs.msg.PointStamped, leica_0_callback)
  sub_vicon_0 = rospy.Subscriber('/camera_imu/vrpn_client/estimated_transform', geometry_msgs.msg.TransformStamped, vicon_0_callback)
  #sub_leica_1 = rospy.Subscriber('/leica/position1', geometry_msgs.msg.PointStamped, leica_1_callback)
  sub_vicon_1 = rospy.Subscriber('/camera_imu/vrpn_client/estimated_transform1', geometry_msgs.msg.TransformStamped, vicon_1_callback)


## Main
if __name__ == '__main__':
  try:
    initROS()
    rospy.spin()

  except rospy.ROSInterruptException:
    pass
