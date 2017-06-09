#!/usr/bin/env python

from collections import OrderedDict
import copy
import select
import sys
import termios
import tty

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64MultiArray

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
        ('la4', {'joint_index': 4, 'side': 'left', 'min': -3.0, 'max': 3.0}),
        ('la5', {'joint_index': 5, 'side': 'left', 'min': -0.6, 'max': 0.6,
               'invert': True}),
        ('la6', {'joint_index': 6, 'side': 'left', 'min': -0.3, 'max': 0.4,
               'invert': True}),
        ('lar', {'joint_index': 'reset', 'side': 'left'}),

        ('ra0', {'joint_index': 0, 'side': 'right', 'min': -3.0, 'max': 3.0}),
        ('ra1', {'joint_index': 1, 'side': 'right', 'min': -3.0, 'max': 3.0}),
        ('ra2', {'joint_index': 2, 'side': 'right', 'min': -3.0, 'max': 3.0}),
        ('ra3', {'joint_index': 3, 'side': 'right', 'min': -0.1, 'max': 2.0}),
        ('ra4', {'joint_index': 4, 'side': 'right', 'min': -3.0, 'max': 3.0}),
        ('ra5', {'joint_index': 5, 'side': 'right', 'min': -0.6, 'max': 0.6}),
        ('ra6', {'joint_index': 6, 'side': 'right', 'min': -0.4, 'max': 0.3}),
        ('rar', {'joint_index': 'reset', 'side': 'right'}),
    ])

    HAND_BINDINGS = OrderedDict([
        ('lh0', {'joint_index': 0, 'side': 'left', 'min': 0.0, 'max': 2.3}),
        ('lht', {'joint_index': 1, 'side': 'left', 'min': -0.9, 'max': 0.0,
               'invert': True}),
        ('lhi', {'joint_index': 2, 'side': 'left', 'min': -1.0, 'max': 0.0,
               'invert': True}),
        ('lhp', {'joint_index': 3, 'side': 'left', 'min': -1.0, 'max': 0.0,
               'invert': True}),
        ('lhr', {'joint_index': 'reset', 'side': 'left'}),

        ('rh0', {'joint_index': 4, 'side': 'right', 'min': 0.0, 'max': 2.3}),
        ('rht', {'joint_index': 5, 'side': 'right', 'min': 0.0, 'max': 1.0}),
        ('rhi', {'joint_index': 6, 'side': 'right', 'min': 0.0, 'max': 0.9}),
        ('rhp', {'joint_index': 7, 'side': 'right', 'min': 0.0, 'max': 0.9}),
        ('rhr', {'joint_index': 'reset', 'side': 'right'}),
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

    def __init__(self,verbose=3):
        self.reset_values = {
            'left arm': [-0.2, -1.4, 0.2, -1.5, 0.0, 0.0, 0.0],
            'right arm': [0.2,  1.4, 0.2,  1.5, 0.0, 0.0, 0.0],
            'hands': [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            'head': [0.0, 0.0, 0.0],
            'neck': [1.08, 0.0, 0.0],
            'chest': [0.0, 0.3, 0.0]
        }
        self.joint_values = copy.deepcopy(self.reset_values)
        self.v = verbose

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
        self.lefthand_publisher = rospy.Publisher(
            '/left_hand_position_controller/command',
            Float64MultiArray, queue_size=1)
        self.righthand_publisher = rospy.Publisher(
            '/right_hand_position_controller/command',
            Float64MultiArray, queue_size=1)
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
        self.lefthand_publisher = None
        self.righthand_publisher = None
        self.neck_publisher = None
        self.head_publisher = None
        self.chest_publisher = None
        self.pelvis_publisher = None

    def process_keys(self, data):  #expects an array of dictionaries
        if len(data)>0:
            if self.v > 2:
                print (data) 
            for d in data:
          	self.process_key(d.key,d) #dictionary {key: str, val: float}

    def process_key(self, ch, data):
        """Process key event."""
        if ch.lower() in self.ARM_BINDINGS:
            self.process_arm_command(self.ARM_BINDINGS[ch.lower()], data)
            return

        if ch.lower() in self.HAND_BINDINGS:
            self.process_hand_command(self.HAND_BINDINGS[ch.lower()], data)
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

    def process_hand_command(self, binding, data):
        msgl = Float64MultiArray()
        msgr = Float64MultiArray()

        self._update_joint_values_d('hands', binding, data)

        #send pinky value to middle and pinky
        arr = [self.joint_values['hands'][i] for i in [0,1,2,3,3]]
        if self.v > 3:
            print(arr) 
        msgl.data = arr
        msgr.data = arr = [self.joint_values['hands'][i] for i in [4,5,6,7,7]]

        self.lefthand_publisher.publish(msgl)
        self.righthand_publisher.publish(msgr)

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
        if self.v > 1:
            rospy.loginfo(msg)

    def read_robot(self,callback):
        t1=['/ihmc_ros/localization/pelvis_odom_pose_correction', 
'/ihmc_ros/localization/pelvis_pose_correction', 
'/ihmc_ros/valkyrie/control/abort_walking', 
'/ihmc_ros/valkyrie/control/arm_desired_joint_accelerations', 
'/ihmc_ros/valkyrie/control/arm_trajectory', 
#5
'/ihmc_ros/valkyrie/control/chest_trajectory', 
'/ihmc_ros/valkyrie/control/end_effector_load_bearing', 
'/ihmc_ros/valkyrie/control/foot_trajectory', 
'/ihmc_ros/valkyrie/control/footstep_list', 
'/ihmc_ros/valkyrie/control/go_home',
#10 
'/ihmc_ros/valkyrie/control/hand_desired_configuration', 
'/ihmc_ros/valkyrie/control/hand_trajectory', 
'/ihmc_ros/valkyrie/control/head_trajectory', 
'/ihmc_ros/valkyrie/control/high_level_state', 
'/ihmc_ros/valkyrie/control/low_level_control_mode', 
#15
'/ihmc_ros/valkyrie/control/neck_desired_acceleration', 
'/ihmc_ros/valkyrie/control/neck_trajectory', 
'/ihmc_ros/valkyrie/control/pause_walking', 
'/ihmc_ros/valkyrie/control/pelvis_height_trajectory', 
'/ihmc_ros/valkyrie/control/pelvis_orientation_trajectory',
#20
'/ihmc_ros/valkyrie/control/pelvis_trajectory', 
'/ihmc_ros/valkyrie/control/request_stop', 
'/ihmc_ros/valkyrie/control/stop_all_trajectories', 
'/ihmc_ros/valkyrie/control/whole_body_trajectory', 
'/ihmc_ros/valkyrie/output/behavior',
#25 
'/ihmc_ros/valkyrie/output/capturability/capture_point', 
'/ihmc_ros/valkyrie/output/capturability/center_of_mass', 
'/ihmc_ros/valkyrie/output/capturability/desired_capture_point', 
'/ihmc_ros/valkyrie/output/capturability/is_in_double_support', 
'/ihmc_ros/valkyrie/output/capturability/left_foot_support_polygon',
#30 
'/ihmc_ros/valkyrie/output/capturability/right_foot_support_polygon', 
'/ihmc_ros/valkyrie/output/foot_force_sensor/left', 
'/ihmc_ros/valkyrie/output/foot_force_sensor/right', 
'/ihmc_ros/valkyrie/output/footstep_status', 
'/ihmc_ros/valkyrie/output/high_level_state',
#35 
'/ihmc_ros/valkyrie/output/high_level_state_change', 
'/ihmc_ros/valkyrie/output/imu/pelvis_pelvisMiddleImu', 
'/ihmc_ros/valkyrie/output/imu/pelvis_pelvisRearImu', 
'/ihmc_ros/valkyrie/output/imu/torso_leftTorsoImu', 
'/ihmc_ros/valkyrie/output/imu/upperNeckPitchLink_head_imu_sensor', 
#40
'/ihmc_ros/valkyrie/output/joint_states', 
'/ihmc_ros/valkyrie/output/robot_motion_status', 
'/ihmc_ros/valkyrie/output/robot_pose', 
'/ihmc_ros/valkyrie/output/walking_status', 
'/imu_states', 
#45
'/leftTorsoImu/imu' ]

#/joint_states -- has hands
#/left_hand_position_controller/command
#/multisense/imu
#/multisense/joint_states  --hokuyo
#/pelvisMiddleImu/imu
#/pelvisRearImu/imu
# 'torsoYaw', 'torsoPitch', 'torsoRoll',
# 'lowerNeckPitch', 'neckYaw', 'upperNeckPitch',
        pass






