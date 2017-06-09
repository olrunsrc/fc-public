#!/usr/bin/env python
from __future__ import print_function

import rospy
import tf
import tf2_ros
import numpy as np
import time

from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry


class Proprio:

    def __init__(self):
	pass

    def init(self):
	if not rospy.has_param('/ihmc_ros/robot_name'):
	    rospy.logerr("Missing parameter '/ihmc_ros/robot_name'")
	else:
	    ROBOT_NAME = rospy.get_param('/ihmc_ros/robot_name')
	    pose_name = "/ihmc_ros/{0}/output/robot_pose".format(ROBOT_NAME)

	    robotPoseSubscriber = rospy.Subscriber( pose_name, Odometry, self.recievedRobotPose )

	    tfBuffer = tf2_ros.Buffer()
	    tfListener = tf2_ros.TransformListener(tfBuffer)

	    rate = rospy.Rate(10) # 10hz
	    time.sleep(1)
	    # make sure the simulation is running otherwise wait
	    if robotPoseSubscriber.get_num_connections() == 0:
	      rospy.loginfo('waiting for subsciber...')
	      while robotPoseSubscriber.get_num_connections() == 0:
		  rate.sleep()

    def fini(self):
	pass

    def loop(self):
	try:
	    rate = rospy.Rate(1.0)
	    #print(globals())
	    while not rospy.is_shutdown():
	      print("Proprioception")
	      rate.sleep()

	except rospy.ROSInterruptException:
	    pass

    def recievedFootStepStatus(self,msg):
	global stepCounter
	if msg.status == 1:
	   stepCounter += 1

    def recievedRobotPose(self,msg):
	global robotpose
	robotpose = msg


if __name__ == '__main__':
    rospy.init_node('proprioception')
    pr = Proprio()
    pr.init()
    pr.loop()
    pr.fini()



def monitor():
  global stepCounter, robotpose, tfBuffer, tfListener, foot_frame
  global ROBOT_NAME, RIGHT_FOOT_FRAME_NAME, LEFT_FOOT_FRAME_NAME
  global getLinkProxy

  time = rospy.Time.now()
  done = False

  pelvispose = getLinkProxy('pelvis','world')
  lfootpose = getLinkProxy('leftFoot','world')
  rfootpose = getLinkProxy('rightFoot','world')
  lfootWorld = tfBuffer.lookup_transform('world', LEFT_FOOT_FRAME_NAME, rospy.Time())
  rfootWorld = tfBuffer.lookup_transform('world', RIGHT_FOOT_FRAME_NAME, rospy.Time())
  l_x = lfootWorld.transform.translation.x

  r_t = robotpose.header.stamp.secs
  r_x = robotpose.pose.pose.position.x
  r_y = robotpose.pose.pose.position.y
  p_x = pelvispose.link_state.pose.position.x
  p_y = pelvispose.link_state.pose.position.y

  os.system('clear')

  #Right foot x,y,rotation (around z) from ros tf2 
  tf_x = rfootWorld.transform.translation.x
  tf_y = rfootWorld.transform.translation.y
  q    = rfootWorld.transform.rotation
  tf_r = euler_from_quaternion([q.x,q.y,q.z,q.w])[2]
  #Right foot x,y,r directly from gazebo
  gz_x = rfootpose.link_state.pose.position.x
  gz_y = rfootpose.link_state.pose.position.y
  q    = rfootpose.link_state.pose.orientation
  gz_r = euler_from_quaternion([q.x,q.y,q.z,q.w])[2]
  print( "time: %6.3f Right tf2: (%6.3f,%6.3f,%6.3f) gazebo: (%6.3f,%6.3f,%6.3f)" %
     (time.to_sec(),tf_x,tf_y,tf_r,gz_x,gz_y,gz_r) )

  #Left foot x,y 
  tf_x = lfootWorld.transform.translation.x
  tf_y = lfootWorld.transform.translation.y
  q    = lfootWorld.transform.rotation
  tf_r = euler_from_quaternion([q.x,q.y,q.z,q.w])[2]
  gz_x = lfootpose.link_state.pose.position.x
  gz_y = lfootpose.link_state.pose.position.y
  q    = lfootpose.link_state.pose.orientation
  gz_r = euler_from_quaternion([q.x,q.y,q.z,q.w])[2]
  print( "steps: %2d    Left tf2: (%6.3f,%6.3f,%6.3f) gazebo: (%6.3f,%6.3f,%6.3f)" %
     (stepCounter,tf_x,tf_y,tf_r,gz_x,gz_y,gz_r) )

  return done, time.to_sec()

def recievedFootStepStatus(msg):
    global stepCounter
    if msg.status == 1:
        stepCounter += 1

def recievedRobotPose(msg):
    global robotpose
    robotpose = msg


