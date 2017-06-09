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

LEFT = 0
RIGHT = 1

ROBOT_NAME = None
LEFT_FOOT_FRAME_NAME = None
RIGHT_FOOT_FRAME_NAME = None

start_line_crossed = None
finish_line_crossed = None

def printhelp(name):
      print(         "%12s -x <dest world dx (0.0m)>" % (name))
      print( "             -y <world dy (0.0m)>")
      print( "             -n <num steps (1)>")
      print( "             -d <dual stance time (1.5s)>")
      print( "             -s <swing time (1.5s)>")
      print( "             -l <arm linger (12s)>")
      print( "             -r <'-r' to extra gazebo>")
      print( "             -e <extra gazebo args>")
      print( "             -b <backup dx (-0.0m)>")
      print( "             -w <stance width (0.16m)>")

def setup(argv):
    global cmdx,cmdy,cmdn,cmdd,cmds,cmde,cmdl,cmdb,cmdw
    copts = "x:y:n:d:s:e:l:b:w:h?r"
    cmdx = 0.0
    cmdy = 0.0
    cmdn = 1
    cmdd = 0.2
    cmds = 0.7
    cmde = ''
    cmdl = 5.0
    cmdb = 0.0
    cmdw = 0.16
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
      if opt == '-l':
        cmdl = float(arg)
      if opt == '-b':
        cmdb = float(arg)
      if opt == '-w':
        cmdw = float(arg)
      if opt == '-r':
        cmde = '-r'
      if opt == '-e':
        cmde = arg

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
    pass #done = True

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
    rospy.init_node('val_runme2')

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
    starttime = rospy.Time.now().to_sec()

    while not rospy.is_shutdown():
      done, curtim = monitor()
      print(backnote,footnote,armnote)
      if curtim > (starttime + 3.0) and not backsent:
        footStepMsg,backnote = createBackStepCmd()
        footStepListPublisher.publish(footStepMsg)
        backsent = True
        print('\a')
      if curtim > (starttime + 10.0) and not armsent:
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
      if curtim > (starttime + 15.0) and not footsent:
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


def createArmCmd():
    global cmdx,cmdy,cmdn,cmdd,cmds,cmde,cmdl,cmdb,cmdw

    #[  'ShP',  'ShR',  'ShY',  'ElP',  'FoY',  'WrR',  'WrP']
    ZERO_VECTOR = [0.0, -1.0, 2.0, 1.0, 0.0, 0.0, 0.0]
    ELBOW_BENT_UP = [0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0]
    REACHING_UP = [0.0, 0.0, -1.5, 2.0, 0.0, 0.0, 0.0]
    #REACH_SWITCH = [-1.35, 0.9, 1.5, 0.0, 0.0, 0.0, 0.0]
    REACH_SWITCH = [-1.35, 0.95, 1.5, 0.0, 0.0, 0.0, 0.0]
    RETRACT = [-1.0, 0.0, 1.5, 1.5, 0.0, 0.0, 0.0]
    HOME_VECTOR = [-0.2, 1.2, 0.7, 1.5, 0.0, 0.0, 0.0]
    ARM_STRAIGHT = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    L_ARM_CLOSE = [-0.2,-1.4, 0.2,-1.5, 0.0, 0.0, 0.0]
    L_ARM_OUT =   [-0.9,-0.4, 0.2,-1.5, 0.0, 0.0, 0.0]
    R_ARM_CLOSE = [-0.2, 1.4, 0.2, 1.5, 0.0, 0.0, 0.0]
    R_ARM_OUT =   [-0.9, 0.4, 0.2, 0.6, 0.0, 0.0, 0.0]

    msgl = ArmTrajectoryRosMessage()
    msgl.robot_side = ArmTrajectoryRosMessage.LEFT
    msgl = appendTrajectoryPoint(msgl, 4.0, L_ARM_OUT)
    msgl = appendTrajectoryPoint(msgl, cmdl+5.0, L_ARM_OUT)
    msgl = appendTrajectoryPoint(msgl, cmdl+5.2, L_ARM_CLOSE)
    msgl.unique_id = -1

    msgr = ArmTrajectoryRosMessage()
    msgr.robot_side = ArmTrajectoryRosMessage.RIGHT
    msgr = appendTrajectoryPoint(msgr, 4.0, R_ARM_OUT)
    msgr = appendTrajectoryPoint(msgr, cmdl+5.0, R_ARM_OUT)
    msgr = appendTrajectoryPoint(msgr, cmdl+5.2, R_ARM_CLOSE)
    msgr.unique_id = -1

    return msgl, msgr, " arms linger for %6.3fs"%(cmdl)

def appendTrajectoryPoint(arm_trajectory, time, positions):
    if not arm_trajectory.joint_trajectory_messages:
        arm_trajectory.joint_trajectory_messages = [copy.deepcopy(OneDoFJointTrajectoryRosMessage()) for i in range(len(positions))]
    for i, pos in enumerate(positions):
        point = TrajectoryPoint1DRosMessage()
        point.time = time
        point.position = pos
        point.velocity = 0
        arm_trajectory.joint_trajectory_messages[i].trajectory_points.append(point)
    return arm_trajectory

def createChestCmd():
    global cmdx,cmdy,cmdn,cmdd,cmds,cmde,cmdl,cmdb,cmdw
    startingside = RIGHT if (cmdy < 0.0) else LEFT

    #Chest roll (never), pitch, yaw
    ZERO_VECTOR = [0.0, 0.0, 0.0]
    LEAN_FWD = [0.0, 0.3, 0.0]
    TURN_IN = [0.0, 0.3, 0.2]

    msg = ChestTrajectoryRosMessage()
    msg = appendTrajectorySO3(msg, 4.0, LEAN_FWD)
    msg = appendTrajectorySO3(msg, cmdl+4.7, LEAN_FWD)
    msg = appendTrajectorySO3(msg, cmdl+4.9, TURN_IN)
    msg = appendTrajectorySO3(msg, cmdl+5.2, LEAN_FWD)
    msg.unique_id = -1

    return msg, " chest"

def appendTrajectorySO3(msg, time, rpy):
    quat = quaternion_from_euler(rpy[0],rpy[1],rpy[2],)
    point = SO3TrajectoryPointRosMessage()
    point.time = time
    point.orientation = Quaternion()
    point.orientation.x = quat[0]
    point.orientation.y = quat[1]
    point.orientation.z = quat[2]
    point.orientation.w = quat[3]
    point.angular_velocity = Vector3()
    point.angular_velocity.x = 0
    point.angular_velocity.y = 0
    point.angular_velocity.z = 0
    msg.taskspace_trajectory_points.append(point)
    return msg

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

def createBackStepCmd():
    global cmdx,cmdy,cmdn,cmdd,cmds,cmde,cmdl
    global tfBuffer, tfListener, stepCounter
    msg = FootstepDataListRosMessage()
    msg.transfer_time = 1.0
    msg.swing_time = 1.0
    msg.execution_mode = 0
    msg.unique_id = -1

    p1 = " back:%6.3f width:%6.3f" % ( cmdb, cmdw)
    offset = (cmdw-0.16)/2.0
    msg.footstep_data_list.append(createFootStepOffset( LEFT, [-cmdb, offset, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset( RIGHT, [-cmdb, -offset, 0.0]))

    return msg,p1

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

if __name__ == '__main__':
  setup(sys.argv)
  main()

