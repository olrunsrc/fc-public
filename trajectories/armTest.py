#!/usr/bin/env python

import copy
import time
import rospy

from numpy import append

from ihmc_msgs.msg import ArmTrajectoryRosMessage
from ihmc_msgs.msg import OneDoFJointTrajectoryRosMessage
from ihmc_msgs.msg import TrajectoryPoint1DRosMessage

ZERO_VECTOR = [0.0, -1.0, 2.0, 1.0, 0.0, 0.0, 0.0]
ELBOW_BENT_UP = [0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0]
REACHING_UP = [0.0, 0.0, -1.5, 2.0, 0.0, 0.0, 0.0]
#REACH_SWITCH = [-1.35, 0.9, 1.5, 0.0, 0.0, 0.0, 0.0]
REACH_SWITCH = [-1.35, 0.95, 1.5, 0.0, 0.0, 0.0, 0.0]
RETRACT = [-1.0, 0.0, 1.5, 1.5, 0.0, 0.0, 0.0]
HOME_VECTOR = [-0.2, 1.2, 0.7, 1.5, 0.0, 0.0, 0.0]
ARM_STRAIGHT = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

#[  'ShP',  'ShR',  'ShY',  'ElP',  'FoY',  'WrR',  'WrP']

#[ -0.199, -1.208, 0.7109, -1.519, -0.005, 0.0020, 0.0009]
#[ -0.199, 1.1984, 0.6993, 1.5001, 0.0007, 0.0035, -0.001]


dwell = 9.7 #14

ROBOT_NAME = None

def sendRightArmTrajectory():
    msg = ArmTrajectoryRosMessage()

    msg.robot_side = ArmTrajectoryRosMessage.RIGHT

    msg = appendTrajectoryPoint(msg, 2.0, ARM_STRAIGHT)
    #msg = appendTrajectoryPoint(msg, 4.0, ELBOW_BENT_UP)
    #msg = appendTrajectoryPoint(msg, 8.0, REACHING_UP)
    #msg = appendTrajectoryPoint(msg, 1.0, RETRACT)
    #msg = appendTrajectoryPoint(msg, 4.5, RETRACT)
    #msg = appendTrajectoryPoint(msg, 2.0, REACH_SWITCH)
    msg = appendTrajectoryPoint(msg, 3.0, REACHING_UP)
    msg = appendTrajectoryPoint(msg, 4.0, REACH_SWITCH)
    msg = appendTrajectoryPoint(msg, dwell+3.0, REACH_SWITCH)
    msg = appendTrajectoryPoint(msg, dwell+3.5, RETRACT)
    msg = appendTrajectoryPoint(msg, dwell+4.0, HOME_VECTOR)

    msg.unique_id = -1

    rospy.loginfo('publishing right trajectory')
    armTrajectoryPublisher.publish(msg)

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

if __name__ == '__main__':
    try:
        rospy.init_node('ihmc_arm_demo1')

        ROBOT_NAME = rospy.get_param('/ihmc_ros/robot_name')

        armTrajectoryPublisher = rospy.Publisher("/ihmc_ros/{0}/control/arm_trajectory".format(ROBOT_NAME), ArmTrajectoryRosMessage, queue_size=1)

        rate = rospy.Rate(10) # 10hz
        time.sleep(1)

        # make sure the simulation is running otherwise wait
        if armTrajectoryPublisher.get_num_connections() == 0:
            rospy.loginfo('waiting for subscriber...')
            while armTrajectoryPublisher.get_num_connections() == 0:
                rate.sleep()

        if not rospy.is_shutdown():
            sendRightArmTrajectory()
            time.sleep(2)

    except rospy.ROSInterruptException:
        pass
