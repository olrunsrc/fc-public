#!/usr/bin/env python

from collections import OrderedDict
import copy
import select
import sys
import termios
import tty

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3

from ihmc_msgs.msg import ArmTrajectoryRosMessage
from ihmc_msgs.msg import HeadTrajectoryRosMessage
from ihmc_msgs.msg import NeckTrajectoryRosMessage
from ihmc_msgs.msg import ChestTrajectoryRosMessage
from ihmc_msgs.msg import PelvisHeightTrajectoryRosMessage
from ihmc_msgs.msg import OneDoFJointTrajectoryRosMessage
from ihmc_msgs.msg import SO3TrajectoryPointRosMessage
from ihmc_msgs.msg import TrajectoryPoint1DRosMessage

import rospy
import yaml

from tf.transformations import quaternion_from_euler

class Axis(object):

    ARM_BINDINGS = OrderedDict([
        ('la0', {'joint_index': 0, 'side': 'left', 'min': -3.0, 'max': 3.0}),
        ('la1', {'joint_index': 1, 'side': 'left', 'min': -3.0, 'max': 3.0,
               'invert': True}),
        ('la2', {'joint_index': 2, 'side': 'left', 'min': -3.0, 'max': 3.0}),
        ('la3', {'joint_index': 3, 'side': 'left', 'min': -2.0, 'max': 0.1,
               'invert': True}),
        ('la4', {'joint_index': 4, 'side': 'left', 'min': -1.0, 'max': 1.0}),
        ('la5', {'joint_index': 5, 'side': 'left', 'min': -0.6, 'max': 0.6,
               'invert': True}),
        ('la6', {'joint_index': 6, 'side': 'left', 'min': -0.3, 'max': 0.4,
               'invert': True}),
        ('lar', {'joint_index': 'reset', 'side': 'left'}),

        ('ra0', {'joint_index': 0, 'side': 'right', 'min': -3.0, 'max': 3.0}),
        ('ra1', {'joint_index': 1, 'side': 'right', 'min': -3.0, 'max': 3.0}),
        ('ra2', {'joint_index': 2, 'side': 'right', 'min': -3.0, 'max': 3.0}),
        ('ra3', {'joint_index': 3, 'side': 'right', 'min': -0.1, 'max': 2.0}),
        ('ra4', {'joint_index': 4, 'side': 'right', 'min': -1.0, 'max': 1.0}),
        ('ra5', {'joint_index': 5, 'side': 'right', 'min': -0.6, 'max': 0.6}),
        ('ra6', {'joint_index': 6, 'side': 'right', 'min': -0.4, 'max': 0.3}),
        ('rar', {'joint_index': 'reset', 'side': 'right'}),
    ])

    HEAD_BINDINGS = OrderedDict([
        ('x', {'joint_index': 0, 'min': -0.5, 'max': 0.5}),
        ('c', {'joint_index': 1, 'min': -0.5, 'max': 0.5}),
        ('v', {'joint_index': 2, 'min': -1.0, 'max': 1.0}),
        ('g', {'joint_index': 'reset'}),
    ])

    NECK_BINDINGS = OrderedDict([
        ('v', {'joint_index': 0, 'min': 0.3, 'max': 1.2}),
        ('n', {'joint_index': 1, 'min': -1.0, 'max': 1.0}),
        ('m', {'joint_index': 2, 'min': -0.5, 'max': 0.0}),
        ('h', {'joint_index': 'reset'}),
    ])

    CHEST_BINDINGS = OrderedDict([
        ('b', {'joint_index': 0, 'min': -0.2, 'max': 0.2}),
        ('n', {'joint_index': 1, 'min': -1.0, 'max': 1.0}),
        ('m', {'joint_index': 2, 'min': -2.5, 'max': 2.5}),
        ('h', {'joint_index': 'reset'}),
    ])

    def __init__(self):
        self.reset_values = {
            'left arm': [-0.2, -1.4, 0.2, -1.5, 0.0, 0.0, 0.0],
            'right arm': [0.2,  1.4, 0.2,  1.5, 0.0, 0.0, 0.0],
            'head': [0.0, 0.0, 0.0],
            'neck': [1.08, 0.0, 0.0],
            'chest': [0.0, 0.3, 0.0],
        }
        self.joint_values = copy.deepcopy(self.reset_values)

    def run(self):
        try:
            self.init()
            while not rospy.is_shutdown():
                ch = self.get_key()
                self.process_key(ch)
        finally:
            self.fini()

    def init(self):

        # create publisher for arm trajectories
        robot_name = rospy.get_param('/ihmc_ros/robot_name')
        self.arm_publisher = rospy.Publisher(
            '/ihmc_ros/{0}/control/arm_trajectory'.format(robot_name),
            ArmTrajectoryRosMessage, queue_size=1)
        self.neck_publisher = rospy.Publisher(
            '/ihmc_ros/{0}/control/neck_trajectory'.format(robot_name),
            NeckTrajectoryRosMessage, queue_size=1)
        self.head_publisher = rospy.Publisher(
            '/ihmc_ros/{0}/control/head_trajectory'.format(robot_name),
            HeadTrajectoryRosMessage, queue_size=1)
        self.chest_publisher = rospy.Publisher(
            '/ihmc_ros/{0}/control/chest_trajectory'.format(robot_name),
            ChestTrajectoryRosMessage, queue_size=1)
        self.pelvis_publisher = rospy.Publisher(
            '/ihmc_ros/{0}/control/pelvis_height_trajectory'.format(robot_name),
            PelvisHeightTrajectoryRosMessage, queue_size=1)

        # make sure the simulation is running otherwise wait
        rate = rospy.Rate(10)  # 10hz
        publishers = [
            self.arm_publisher, self.neck_publisher, self.head_publisher, self.pelvis_publisher]
        if any([p.get_num_connections() == 0 for p in publishers]):
            rospy.loginfo('waiting for subscriber...')
            while any([p.get_num_connections() == 0 for p in publishers]):
                rate.sleep()

    def fini(self):
        self.arm_publisher = None
        self.neck_publisher = None
        self.head_publisher = None
        self.chest_publisher = None
        self.pelvis_publisher = None

    def process_keys(self, data):
        if len(data)>0:
            print (data)
            for d in data:
          	self.process_key(d.key,d)

    def process_key(self, ch, data):
        """Process key event."""
        if ch.lower() in self.ARM_BINDINGS:
            self.process_arm_command(self.ARM_BINDINGS[ch.lower()], data)
            return

        if ch.lower() in self.HEAD_BINDINGS:
            self.process_head_command(self.HEAD_BINDINGS[ch.lower()], ch)
            return

        if ch.lower() in self.NECK_BINDINGS:
            self.process_neck_command(self.NECK_BINDINGS[ch.lower()], ch)
            return

        if ch.lower() in self.CHEST_BINDINGS:
            self.process_chest_command(self.CHEST_BINDINGS[ch.lower()], ch)
            return

    def process_arm_command(self, binding, data):
        msg = ArmTrajectoryRosMessage()
        msg.unique_id = -1

        side = binding['side']
        if side == 'left':
            msg.robot_side = ArmTrajectoryRosMessage.LEFT
        elif side == 'right':
            msg.robot_side = ArmTrajectoryRosMessage.RIGHT
        else:
            assert False, "Unknown arm side '%s'" % side

        self._update_joint_values_d('%s arm' % side, binding, data)
	if binding['joint_index']=='reset': time = 1.0
	else: time = data.get_sibling("tim").get_rosval()
        self._append_trajectory_point_1d(
            msg, time, self.joint_values['%s arm' % side])

        self.arm_publisher.publish(msg)

    def process_head_command(self, binding, ch):
        msg = HeadTrajectoryRosMessage()
        msg.unique_id = -1
        self._update_joint_values('head', binding, ch)
        self._append_trajectory_point_so3(
            msg, 1.0, self.joint_values['head'])
        self.head_publisher.publish(msg)

    #def process_neck_command(self, binding, ch):
    #    msg = NeckTrajectoryRosMessage()
    #    msg.unique_id = -1
    #    self._update_joint_values('neck', binding, ch)
    #    self._append_trajectory_point_1d(
    #        msg, 1.0, self.joint_values['neck'])
    #    self.neck_publisher.publish(msg)
    def process_neck_command(self, binding, ch):
        msg = PelvisHeightTrajectoryRosMessage()
        msg.unique_id = -1
        self._update_joint_values('neck', binding, ch)
        self._append_single_trajectory_point_1d(
            msg, 1.0, self.joint_values['neck'][0])
        self.pelvis_publisher.publish(msg)

    def process_chest_command(self, binding, ch):
        msg = ChestTrajectoryRosMessage()
        msg.unique_id = -1
        self._update_joint_values('chest', binding, ch)
        self._append_trajectory_point_so3(
            msg, 1.0, self.joint_values['chest'])
        self.chest_publisher.publish(msg)

    def _update_joint_values(self, label, binding, ch):
        joint_index = binding['joint_index']
        if joint_index == 'reset':
            # reset all joints to zero
            self.loginfo('Reset %s' % label)
            #self.joint_values[label] = [0.0] * len(self.joint_values[label])
            self.joint_values[label] = copy.deepcopy(self.reset_values[label])

        else:
            # update value within boundaries
            value = self.joint_values[label][joint_index]
            is_lower_case = ch == ch.lower()
            factor = 1.0 if is_lower_case else -1.0
            if binding.get('invert', False):
                factor *= -1.0
            STEP = 0.1
            value += STEP * factor
            value = min(max(value, binding['min']), binding['max'])
            self.joint_values[label][joint_index] = value

            self.loginfo(
                'Move %s joint #%d %s to %.3f: [%s]' %
                (label, joint_index, '++' if is_lower_case else '--',
                 self.joint_values[label][joint_index],
                 ', '.join(
                     ['%.1f' % v for v in self.joint_values[label]])))

    def _update_joint_values_d(self, label, binding, data):
        joint_index = binding['joint_index']
        if joint_index == 'reset':
            # reset all joints
            self.loginfo('Reset %s' % label)
            self.joint_values[label] = copy.deepcopy(self.reset_values[label])
        else:
            # update value within boundaries
            value = data.get_rosval()
            value = min(max(value, binding['min']), binding['max'])
            self.joint_values[label][joint_index] = value

            self.loginfo(
                'Move %s joint #%d %s to %.3f: [%s]' %
                (label, joint_index, ' ',
                 self.joint_values[label][joint_index],
                 ', '.join(
                     ['%.1f' % v for v in self.joint_values[label]])))


    def _append_trajectory_point_1d(self, msg, time, joint_values):
        if not msg.joint_trajectory_messages:
            msg.joint_trajectory_messages = [
                OneDoFJointTrajectoryRosMessage() for _ in joint_values]
        for i, joint_value in enumerate(joint_values):
            point = TrajectoryPoint1DRosMessage()
            point.time = time
            point.position = joint_value
            point.velocity = 0
            msg.joint_trajectory_messages[i].trajectory_points.append(point)

    def _append_single_trajectory_point_1d(self, msg, time, joint_value):
        if not msg.trajectory_points:
            msg.trajectory_points = []
            point = TrajectoryPoint1DRosMessage()
            point.time = time
            point.position = joint_value
            point.velocity = 0
            msg.trajectory_points.append(point)

    def _append_trajectory_point_so3(self, msg, time, joint_values):
        roll, pitch, yaw = joint_values
        quat = quaternion_from_euler(roll, pitch, yaw)
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

    def loginfo(self, msg):
        rospy.loginfo(msg)





