#!/usr/bin/env python
from __future__ import print_function

import copy
import time
import rospy
import tf
import tf2_ros
import numpy as np
import os, sys, getopt
import math
import threading
import signal
import subprocess as prc

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry

from ihmc_msgs.msg import FootstepStatusRosMessage
from ihmc_msgs.msg import FootstepDataListRosMessage
from ihmc_msgs.msg import FootstepDataRosMessage
from ihmc_msgs.msg import ArmTrajectoryRosMessage
from ihmc_msgs.msg import ChestTrajectoryRosMessage
from ihmc_msgs.msg import OneDoFJointTrajectoryRosMessage
from ihmc_msgs.msg import TrajectoryPoint1DRosMessage
from ihmc_msgs.msg import SO3TrajectoryPointRosMessage
from tf.transformations import quaternion_from_euler

from addasteppages import Pages, Qitem
import thread
import threading
import Queue

LEFT = 0
RIGHT = 1

ROBOT_NAME = None
LEFT_FOOT_FRAME_NAME = None
RIGHT_FOOT_FRAME_NAME = None

start_line_crossed = None
finish_line_crossed = None

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
  if ( time.to_sec() > 160.0 or
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

  global web
  web = addasteppages.Pages()
  thread.start_new_thread(web.run, ())

  try:
    rospy.init_node('AddAStep')

    if not rospy.has_param('/ihmc_ros/robot_name'):
      rospy.logerr("Missing parameter '/ihmc_ros/robot_name'")
    else:
      ROBOT_NAME = rospy.get_param('/ihmc_ros/robot_name')
      right_foot_frame_parameter_name = "/ihmc_ros/{0}/right_foot_frame_name".format(ROBOT_NAME)
      left_foot_frame_parameter_name = "/ihmc_ros/{0}/left_foot_frame_name".format(ROBOT_NAME)
      fss_name = "/ihmc_ros/{0}/output/footstep_status".format(ROBOT_NAME)
      fsp_name = "/ihmc_ros/{0}/control/footstep_list".format(ROBOT_NAME)
      atp_name = "/ihmc_ros/{0}/control/arm_trajectory".format(ROBOT_NAME)
      ctp_name = "/ihmc_ros/{0}/control/chest_trajectory".format(ROBOT_NAME)
      pose_name = "/ihmc_ros/{0}/output/robot_pose".format(ROBOT_NAME)

    if rospy.has_param(right_foot_frame_parameter_name) and rospy.has_param(left_foot_frame_parameter_name):
      RIGHT_FOOT_FRAME_NAME = rospy.get_param(right_foot_frame_parameter_name)
      LEFT_FOOT_FRAME_NAME = rospy.get_param(left_foot_frame_parameter_name)

    footStepStatusSubscriber = rospy.Subscriber( fss_name, FootstepStatusRosMessage, recievedFootStepStatus)
    robotPoseSubscriber = rospy.Subscriber( pose_name, Odometry, recievedRobotPose )
    footStepListPublisher = rospy.Publisher( fsp_name, FootstepDataListRosMessage, queue_size=1)
    armTrajectoryPublisher = rospy.Publisher( atp_name, ArmTrajectoryRosMessage, queue_size=1)
    chestTrajectoryPublisher = rospy.Publisher( ctp_name, ChestTrajectoryRosMessage, queue_size=1)

    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10) # 10hz
    time.sleep(1)

    # make sure the simulation is running otherwise wait
    if footStepListPublisher.get_num_connections() == 0:
      rospy.loginfo('waiting for subsciber...')
      while footStepListPublisher.get_num_connections() == 0:
        rate.sleep()

    stepCounter = 0
    backsent = armsent = footsent = False
    backnote = armnote = footnote = 'Wait '
    while not rospy.is_shutdown():
      done, curtim = monitor()
      print(backnote,footnote,armnote)
      if curtim > 28.0 and not backsent:
        footStepMsg,backnote = createBackStepCmd()
        footStepListPublisher.publish(footStepMsg)
        backsent = True
        print('\a')
      if curtim > 35.0 and not armsent:
        footStepMsg,footnote = createFootStepCmd()
        armMsgL, armMsgR, armnote = createArmCmd()
        armTrajectoryPublisher.publish(armMsgL)
        rospy.sleep(rospy.Duration(0,10000000))
        armTrajectoryPublisher.publish(armMsgR)
        chestMsg, _ = createChestCmd()
        chestTrajectoryPublisher.publish(chestMsg)
        #armTrajectoryPublisher.publish(armMsgR)
        armsent = True
        print('\a')
      if curtim > 40.0 and not footsent:
        footStepListPublisher.publish(footStepMsg)
        footsent = True
        print('\a')
        print('\a')
      if (done):
        print('\a')
        print('\a')
        return
      rate.sleep()

  except rospy.ROSInterruptException:
        pass


def createFootStepCmd():
    global cmdx,cmdy,cmdn,cmdd,cmds,cmde,cmdl
    global tfBuffer, tfListener, stepCounter
    msg = FootstepDataListRosMessage()
    msg.transfer_time = float(cmdd)
    msg.swing_time = float(cmds)
    msg.execution_mode = 0
    msg.unique_id = -1

    # walk forward starting RIGHT if y is negative
    # each full step must be <0.75x and <0.2y
    startingside = RIGHT if (cmdy < 0.0) else LEFT
    halfsteps = max(cmdn,1)
    #print( halfsteps )
    halfsteps = int(math.ceil(abs(cmdx)/0.6)) if (abs(cmdx)/halfsteps > 0.75) else halfsteps
    #print( halfsteps )
    halfsteps = int(math.ceil(abs(cmdy)/0.2)) if (abs(cmdy)/halfsteps > 0.25) else halfsteps
    stridex = cmdx/halfsteps
    stridey = cmdy/halfsteps

    p1 = "n:%2d dx:%6.3f dy:%6.3f" % ( halfsteps, stridex, stridey )
    p2 = " l/r:%2d dual:%6.3f swing:%6.3f" % ( startingside, cmdd, cmds)
    #print( p1 + p2)

    msg.footstep_data_list.append(createFootStepOffset( startingside, [stridex, 2.0*stridey, 0.0]))
    for step in range(1,halfsteps-1):
      side = (startingside+step) % 2
      msg.footstep_data_list.append(createFootStepOffset(side, [stridex*(step+1), 2.0*stridey*step, 0.0]))
    if halfsteps > 1:
      step = halfsteps -1
      side = (startingside+step) % 2
      msg.footstep_data_list.append(createFootStepOffset(side, [stridex*(step+1), cmdy, 0.0]))
    if stridex != 0.0:
      msg.footstep_data_list.append(createFootStepOffset((startingside+halfsteps)%2, [cmdx, cmdy, 0.0]))
    return msg,p1+p2    

# Creates footstep with the current position and orientation of the foot.
def createFootStepInPlace(stepSide):
    global tfBuffer, tfListener, stepCounter
    global ROBOT_NAME, RIGHT_FOOT_FRAME_NAME, LEFT_FOOT_FRAME_NAME
    footstep = FootstepDataRosMessage()
    footstep.robot_side = stepSide

    if stepSide == LEFT:
        foot_frame = LEFT_FOOT_FRAME_NAME
    else:
        foot_frame = RIGHT_FOOT_FRAME_NAME

    footWorld = tfBuffer.lookup_transform('world', foot_frame, rospy.Time())
    footstep.orientation = footWorld.transform.rotation
    footstep.location = footWorld.transform.translation

    return footstep

# Creates footstep offset from the current foot position. The offset is in foot frame.
def createFootStepOffset(stepSide, offset):
    footstep = createFootStepInPlace(stepSide)

    # transform the offset to world frame
    quat = footstep.orientation
    rot = tf.transformations.quaternion_matrix([quat.x, quat.y, quat.z, quat.w])
    transformedOffset = np.dot(rot[0:3, 0:3], offset)

    footstep.location.x += transformedOffset[0]
    footstep.location.y += transformedOffset[1]
    footstep.location.z += transformedOffset[2]

    return footstep

class Smartie():
    def __init__(self):
        self.queue = Queue.Queue()
        self.data = 0

    def loop(self):
        self.data += 1
        if not self.queue.empty():
            item = self.queue.get()
            self.data += 1
            print( "Queued data is %s" % item.req )
            item.resp = self.data
            item.evt.set()
            #item.evt.clear()
            self.queue.task_done()

def mainish():
    rospy.init_node('AddAStep')
    theSmartie = Smartie()
    web = Pages(theSmartie)
    rate = rospy.Rate(10) # 10hz
    time.sleep(1)
    try:
	    thread.start_new_thread(web.run, ())
            while not rospy.is_shutdown():
		theSmartie.loop()
                rate.sleep()
    finally:
            rospy.loginfo("Smartie Died")

if __name__ == '__main__':
  mainish()

