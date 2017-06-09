#!/usr/bin/env python
from __future__ import print_function

import time
import rospy
import tf
import tf2_ros
import numpy
import os, sys, getopt
import math
import threading
import signal
import subprocess as prc

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

def monitor():
  global stepCounter, robotpose, tfBuffer, tfListener, foot_frame
  global ROBOT_NAME, RIGHT_FOOT_FRAME_NAME, LEFT_FOOT_FRAME_NAME
  global start_line_crossed, finish_line_crossed

  time = rospy.Time.now()
  done = False

  foot_frame = LEFT_FOOT_FRAME_NAME
  lfootWorld = tfBuffer.lookup_transform('world', LEFT_FOOT_FRAME_NAME, rospy.Time())
  rfootWorld = tfBuffer.lookup_transform('world', RIGHT_FOOT_FRAME_NAME, rospy.Time())
  l_x = lfootWorld.transform.translation.x
  r_x = rfootWorld.transform.translation.x
  r_t = robotpose.header.stamp.secs
  r_x = robotpose.pose.pose.position.x
  r_z = robotpose.pose.pose.position.z

  if( r_x >= 0.5 and not start_line_crossed ):
    start_line_crossed = time
  elif( r_x >= 4.5 and not finish_line_crossed ):
    finish_line_crossed = time

  os.system('clear')
  print( "time: %6.3f steps: %2d robot: (%6.3f,%6.3f) left: %6.3f right: %6.3f" %
     (time.to_sec(),stepCounter,r_x,r_z,l_x,r_x) )
  if( start_line_crossed ):
    print( " Started: %6.3f" % (start_line_crossed.to_sec()) )
  if( finish_line_crossed ):
    print( "Finished: %6.3f" % (finish_line_crossed.to_sec()) )
    print( " Elapsed: %6.3f" % ((finish_line_crossed - start_line_crossed).to_sec()) )
  if ( time.to_sec() > 60.0 or
       r_x > 5.0 or
       r_z < 0.5  ):
    done = True

  return done, time.to_sec()

def recievedFootStepStatus(msg):
    global stepCounter
    if msg.status == 1:
        stepCounter += 1

def recievedRobotPose(msg):
    global robotpose
    robotpose = msg

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
      done, curtim = monitor()
      if (done):
        return
      rate.sleep()

  except rospy.ROSInterruptException:
        pass

def launchval(extra=''):
  vallaunch = 'roslaunch srcsim qual2.launch init:=true ' + extra
  val = prc.Popen(vallaunch, shell=True )
  return val

def findlaunchpid(pid):
  ef = prc.check_output('ps -f', shell=True)
  #print(ef)
  lines = [[ a for a in b.split(' ') if len(a)>0 ] for b in ef.split('\n') ]
  #print(lines)
  procs = [ a[1] for a in lines if len(a)>6 and a[2] == str(pid) and a[7] == '/usr/bin/python']
  #print(procs)
  if (len(procs)==1):
    return procs[0]
  else:
    print(" No process found - procs = %s " % (procs) )

def ctrlc(pid):
  print( "Cancelling process %s" % pid )
  prc.call('kill -2 '+str(pid),shell=True)
  

if __name__ == '__main__':
  setup(sys.argv)
  val = launchval()
  time.sleep(30) #more that enough to get the clock running
  main()
  shellpid = val.pid
  print(shellpid)
  rospid = findlaunchpid(shellpid)
  ctrlc(rospid)
