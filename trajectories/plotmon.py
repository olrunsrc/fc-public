#!/usr/bin/env python
from __future__ import print_function

import time
import rospy
import tf
import tf2_ros
import numpy as np
import os, sys, getopt
import math

import matplotlib.pyplot as plt
import matplotlib.animation as ani

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry

from ihmc_msgs.msg import FootstepStatusRosMessage
from ihmc_msgs.msg import FootstepDataRosMessage

LEFT = 0
RIGHT = 1

ROBOT_NAME = None
LEFT_FOOT_FRAME_NAME = None
RIGHT_FOOT_FRAME_NAME = None

start_line_crossed = None
finish_line_crossed = None

def printhelp(name):
      print(         "%12s -x <dest world x (0.0m)>" % (name))
      print( "             -y <world y (0.0m)>")
      print( "             -n <num steps (1)>")
      print( "             -d <dual stance time (1.5s)>")
      print( "             -s <swing time (1.5s)>")

def setup(argv):
    global fig1,ax1,pltdata

    copts = "x:y:n:d:s:h?"
    cmdx = 0.0
    cmdy = 0.0
    cmdn = 1
    cmdd = 1.5
    cmds = 1.5
    try:
      opts, args = getopt.getopt(argv[1:],copts)
    except getopt.GetoptError:
      printhelp(argv[0])
      sys.exit(2)
    for opt, arg in opts:
      if opt in ('-h','-?'):
        printhelp(argv[0])
      if opt == '-x':
        cmdx = float(arg)
      if opt == '-y':
        cmdy = float(arg)
      if opt == '-n':
        cmdn = int(arg)
      if opt == '-d':
        cmdd = float(arg)
      if opt == '-s':
        cmds = float(arg)

    fig1 = plt.figure()
    ax1 = fig1.add_subplot(1,1,1)
    pltdata = np.zeros((5,300),dtype=np.float32) #30sec @ 10Hz (time,left,right,com_x,com_z)

    pltdata[0] = np.linspace(30,60,300)
    pltdata[1] = np.sin(pltdata[0])
    pltdata[2] = np.cos(pltdata[0])
    pltdata[3] = 0.2*pltdata[0]-6.0
    pltdata[4] = 0.1*pltdata[0]-2.0

    g = ani.FuncAnimation(fig1, animate, interval=500)
    plt.show()

def monitor():
  global stepCounter, robotpose, tfBuffer, tfListener, foot_frame
  global ROBOT_NAME, RIGHT_FOOT_FRAME_NAME, LEFT_FOOT_FRAME_NAME
  global start_line_crossed, finish_line_crossed
  global fig1,ax1,pltdata

  time = rospy.Time.now()


  foot_frame = LEFT_FOOT_FRAME_NAME
  footWorld = tfBuffer.lookup_transform('world', foot_frame, rospy.Time())
  footstep = FootstepDataRosMessage()
  footstep.orientation = footWorld.transform.rotation
  footstep.location = footWorld.transform.translation
  f_x = footstep.location.x
  r_t = robotpose.header.stamp.secs
  r_x = robotpose.pose.pose.position.x

  if( r_x >= 0.5 and not start_line_crossed ):
    start_line_crossed = time
  elif( r_x >= 4.5 and not finish_line_crossed ):
    finish_line_crossed = time

  os.system('clear')
  print( "time: %6.3f steps: %2d robot_x: %6.3f l_foot_x: %6.3f" % (time.to_sec(),stepCounter,r_x,f_x) )
  if( start_line_crossed ):
    print( " Started: %6.3f" % (start_line_crossed.to_sec()) )
  if( finish_line_crossed ):
    print( "Finished: %6.3f" % (finish_line_crossed.to_sec()) )
    print( " Elapsed: %6.3f" % ((finish_line_crossed - start_line_crossed).to_sec()) )



# Creates footstep offset from the current foot position. The offset is in foot frame.
def createFootStepOffset(stepSide, offset):
    footstep = createFootStepInPlace(stepSide)

    # transform the offset to world frame
    quat = footstep.orientation
    rot = tf.transformations.quaternion_matrix([quat.x, quat.y, quat.z, quat.w])
    transformedOffset = numpy.dot(rot[0:3, 0:3], offset)

    footstep.location.x += transformedOffset[0]
    footstep.location.y += transformedOffset[1]
    footstep.location.z += transformedOffset[2]

    return footstep

def recievedFootStepStatus(msg):
    global stepCounter
    if msg.status == 1:
        stepCounter += 1

def recievedRobotPose(msg):
    global robotpose
    robotpose = msg

def animate(i):
    global fig1,ax1,pltdata
    ax1.clear()
    ax1.plot(pltdata[0],pltdata[1],'b-')
    ax1.plot(pltdata[0],pltdata[2],'g-')
    ax1.plot(pltdata[0],pltdata[3],'b:')
    ax1.plot(pltdata[0],pltdata[4],'r-')

def main():
  global tfBuffer, tfListener, stepCounter
  global ROBOT_NAME, RIGHT_FOOT_FRAME_NAME, LEFT_FOOT_FRAME_NAME
  try:
    rospy.init_node('val_monitor')

    if not rospy.has_param('/ihmc_ros/robot_name'):
      rospy.logerr("Missing parameter '/ihmc_ros/robot_name'")
    else:
      ROBOT_NAME = rospy.get_param('/ihmc_ros/robot_name')
      right_foot_frame_parameter_name = "/ihmc_ros/{0}/right_foot_frame_name".format(ROBOT_NAME)
      left_foot_frame_parameter_name = "/ihmc_ros/{0}/left_foot_frame_name".format(ROBOT_NAME)
      fss_name = "/ihmc_ros/{0}/output/footstep_status".format(ROBOT_NAME)
      pose_name = "/ihmc_ros/{0}/output/robot_pose".format(ROBOT_NAME)

    if rospy.has_param(right_foot_frame_parameter_name) and rospy.has_param(left_foot_frame_parameter_name):
      RIGHT_FOOT_FRAME_NAME = rospy.get_param(right_foot_frame_parameter_name)
      LEFT_FOOT_FRAME_NAME = rospy.get_param(left_foot_frame_parameter_name)

    footStepStatusSubscriber = rospy.Subscriber( fss_name, FootstepStatusRosMessage, recievedFootStepStatus)
    robotPoseSubscriber = rospy.Subscriber( pose_name, Odometry, recievedRobotPose )

    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10) # 10hz
    time.sleep(1)

    stepCounter = 0
    while not rospy.is_shutdown():
      monitor()
      rate.sleep()

  except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
  setup(sys.argv)
  main()
