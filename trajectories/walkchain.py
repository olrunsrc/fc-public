#!/usr/bin/env python

import time
import rospy
import tf
import tf2_ros
import numpy
import sys, getopt
import math

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion

from ihmc_msgs.msg import FootstepStatusRosMessage
from ihmc_msgs.msg import FootstepDataListRosMessage
from ihmc_msgs.msg import FootstepDataRosMessage

LEFT = 0
RIGHT = 1

ROBOT_NAME = None
LEFT_FOOT_FRAME_NAME = None
RIGHT_FOOT_FRAME_NAME = None

def printhelp(name):
      print         "%12s -x <dest world x (0.0m)>"%(name)
      print "             -y <world y (0.0m)>"
      print "             -n <num steps (1)>"
      print "             -d <dual stance time (1.5s)>"
      print "             -s <swing time (1.5s)>"

def walkTest(argv):
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

    msg = FootstepDataListRosMessage()
    msg.transfer_time = float(cmdd)
    msg.swing_time = float(cmds)
    msg.execution_mode = 0
    msg.unique_id = -1

    # walk forward starting RIGHT if y is negative
    # each full step must be <0.75x and <0.2y
    startingside = RIGHT if (cmdy < 0.0) else LEFT
    halfsteps = max(cmdn,1)
    print halfsteps
    halfsteps = int(math.ceil(abs(cmdx)/0.5)) if (abs(cmdx)/halfsteps > 0.5) else halfsteps
    print halfsteps
    halfsteps = int(math.ceil(abs(cmdy)/0.2)) if (abs(cmdy)/halfsteps > 0.2) else halfsteps
    stridex = cmdx/halfsteps
    stridey = cmdy/halfsteps

    p1 = "n:%2d dx:%6.3f dy:%6.3f" % ( halfsteps, stridex, stridey )
    p2 = " l/r:%2d dual:%4.1s swing:%4.1s" % ( startingside, cmdd, cmds)
    print p1 + p2

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
    
    footStepListPublisher.publish(msg)
    rospy.loginfo('walking...')
    waitForFootsteps(len(msg.footstep_data_list))

# Creates footstep with the current position and orientation of the foot.
def createFootStepInPlace(stepSide):
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
    transformedOffset = numpy.dot(rot[0:3, 0:3], offset)

    footstep.location.x += transformedOffset[0]
    footstep.location.y += transformedOffset[1]
    footstep.location.z += transformedOffset[2]

    return footstep

def waitForFootsteps(numberOfSteps):
    global stepCounter
    stepCounter = 0
    while stepCounter < numberOfSteps:
        rate.sleep()
    rospy.loginfo('finished set of steps')

def recievedFootStepStatus(msg):
    global stepCounter
    if msg.status == 1:
        stepCounter += 1

if __name__ == '__main__':
    try:
        rospy.init_node('ihmc_walk_test')

        if not rospy.has_param('/ihmc_ros/robot_name'):
            rospy.logerr("Cannot run walkcmd.py, missing parameters!")
            rospy.logerr("Missing parameter '/ihmc_ros/robot_name'")

        else:
            ROBOT_NAME = rospy.get_param('/ihmc_ros/robot_name')

            right_foot_frame_parameter_name = "/ihmc_ros/{0}/right_foot_frame_name".format(ROBOT_NAME)
            left_foot_frame_parameter_name = "/ihmc_ros/{0}/left_foot_frame_name".format(ROBOT_NAME)

            if rospy.has_param(right_foot_frame_parameter_name) and rospy.has_param(left_foot_frame_parameter_name):
                RIGHT_FOOT_FRAME_NAME = rospy.get_param(right_foot_frame_parameter_name)
                LEFT_FOOT_FRAME_NAME = rospy.get_param(left_foot_frame_parameter_name)

                footStepStatusSubscriber = rospy.Subscriber("/ihmc_ros/{0}/output/footstep_status".format(ROBOT_NAME), FootstepStatusRosMessage, recievedFootStepStatus)
                footStepListPublisher = rospy.Publisher("/ihmc_ros/{0}/control/footstep_list".format(ROBOT_NAME), FootstepDataListRosMessage, queue_size=1)

                tfBuffer = tf2_ros.Buffer()
                tfListener = tf2_ros.TransformListener(tfBuffer)

                rate = rospy.Rate(10) # 10hz
                time.sleep(1)

                # make sure the simulation is running otherwise wait
                if footStepListPublisher.get_num_connections() == 0:
                    rospy.loginfo('waiting for subsciber...')
                    while footStepListPublisher.get_num_connections() == 0:
                        rate.sleep()

                if not rospy.is_shutdown():
                    walkTest(sys.argv)
            else:
                if not rospy.has_param(left_foot_frame_parameter_name):
                    rospy.logerr("Missing parameter {0}".format(left_foot_frame_parameter_name))
                if not rospy.has_param(right_foot_frame_parameter_name):
                    rospy.logerr("Missing parameter {0}".format(right_foot_frame_parameter_name))

    except rospy.ROSInterruptException:
        pass
