#!/usr/bin/env python
from __future__ import print_function

from collections import OrderedDict
from math import cos, radians, sin, sqrt
import select
import sys
import termios
import tty

from geometry_msgs.msg import Quaternion, Transform, Vector3

from ihmc_msgs.msg import AbortWalkingRosMessage
from ihmc_msgs.msg import ArmTrajectoryRosMessage
from ihmc_msgs.msg import FootstepDataListRosMessage
from ihmc_msgs.msg import FootstepDataRosMessage
from ihmc_msgs.msg import FootstepStatusRosMessage
from ihmc_msgs.msg import HeadTrajectoryRosMessage
from ihmc_msgs.msg import NeckTrajectoryRosMessage
from ihmc_msgs.msg import OneDoFJointTrajectoryRosMessage
from ihmc_msgs.msg import SO3TrajectoryPointRosMessage
from ihmc_msgs.msg import TrajectoryPoint1DRosMessage

import numpy as np
import rospy

from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix
import tf2_ros

S_W = 0.2 #stance width
SW2 = S_W/2.0
D1  = np.pi/180
D15 = 15*D1

class Foot:
    LEFT = 1
    RIGHT = -1
    def __init__(self,xyt,side):
        self.x = xyt[0]
        self.y = xyt[1]
        self.t = xyt[2]
        self.side = side #Left +1, Right -1

class Center:
    def __init__(self,foot,bigR):
        self.foot = foot
        self.r = bigR
        l = bigR - foot.side*SW2
        self.x = foot.x - l*np.sin(foot.t)
        self.y = foot.y + l*np.cos(foot.t)
        
    def newFoot(self,th):
        side = -1*self.foot.side
        theta = self.foot.t + th
        l = self.r - side*SW2
        x = self.x + l*np.sin(theta)
        y = self.y - l*np.cos(theta)
        return Foot([x,y,theta],side)
        
class Steplist:
    def __init__(self,first,other):
        self.list=[other,first]
        self.center=[]
        self.complete = 1
        self.current = None
        
    def step(self,R,th):
	c = Center(self.list[-1],R)
        f = c.newFoot(th)
        self.center.append(c)
        self.list.append(f)
	return f        

    def stepn(self,R,th,n=1):
        for i in range(n):
            self.step(R,th)
        self.step(R,0)

class Astep():
    def __init__(self,k,v,r=3,c=3):
        self.key = k
        self.val = v
        self.row = r
        self.col = c
    def __repr__(self):
        return "!%s(%d,%d) %s = %d" % (self.__class__.__name__,
            self.row,self.col,self.key,self.val)
    def get_rosval(self):
        return self.val
    def get_sibling(self,idx):
        return Astep(idx,0.5)

class Pose():
    def __init__(self,transform=Transform()):
       self.hcoord = 0

class Frame():
    def __init__(self):
        self.offset = Pose()
        self.planned = Pose()
        self.est = Pose()
        self.act = Pose()

class Walker():
    # constants used for walking
    LEFT_FOOT_FRAME_NAME = None
    RIGHT_FOOT_FRAME_NAME = None
    TRANS_STEP = 0.2  # each step will be 20cm
    ROT_STEP = radians(45)  # each rotation will be 45degrees
    WALKING_BINDINGS = OrderedDict([
        ('f', {'action': 'translate'}),
        ('z', {'action': 'rotate'}),
    ])

    def receivedFootStepStatus_cb(self, msg):
        if msg.status == FootstepStatusRosMessage.COMPLETED:
            self.footstep_count += 1

    def __init__(self,verbose=3,logger=print):
        self.v = verbose
        self.loginfo=logger

    def init(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)  # noqa
        self.footstep_count = 0
        robot_name = rospy.get_param('/ihmc_ros/robot_name')
        self.footstep_publisher = rospy.Publisher(
            '/ihmc_ros/{0}/control/footstep_list'.format(robot_name),
            FootstepDataListRosMessage, queue_size=1)
        self.footstep_status_subscriber = rospy.Subscriber(
            '/ihmc_ros/{0}/output/footstep_status'.format(robot_name),
            FootstepStatusRosMessage, self.receivedFootStepStatus_cb)
        self.abort_walking_publisher = rospy.Publisher(
            '/ihmc_ros/{0}/control/abort_walking'.format(robot_name),
            AbortWalkingRosMessage, queue_size=1)

        right_foot_frame_parameter_name = "/ihmc_ros/{0}/right_foot_frame_name".format(robot_name)
        left_foot_frame_parameter_name = "/ihmc_ros/{0}/left_foot_frame_name".format(robot_name)
        if rospy.has_param(right_foot_frame_parameter_name) and \
                rospy.has_param(left_foot_frame_parameter_name):
            self.RIGHT_FOOT_FRAME_NAME = rospy.get_param(right_foot_frame_parameter_name)
            self.LEFT_FOOT_FRAME_NAME = rospy.get_param(left_foot_frame_parameter_name)
        self.rate = rospy.Rate(10)  # 10hz
        self.recentz = 0
        firstFoot = self.createFoot(Foot.RIGHT)
        standingFoot = self.createFoot(Foot.LEFT)
        self.list = Steplist(firstFoot,standingFoot)

    def process_keys(self, data):  #expects an array of dictionaries
        if len(data)>0:
            if self.v > 2:
                print (data) 
            for d in data:
          	self.process_key(d.key,d) #dictionary {key: str, val: float}

    def process_key(self, ch, data):
        """Process key event."""
        if ch.lower() in self.WALKING_BINDINGS:
            self.process_walking_command(self.WALKING_BINDINGS[ch.lower()], ch)
            return
        if ch.lower() == 'rst':   #all walker cmds = rst-reset,wlk,sup-stepup,tlt-torso,nck-[3],abt
            return
        if ch.lower() == 'wlk':   #all walker cmds = rst-reset,wlk,sup-stepup,tlt-torso,nck-[3],abt
            self.process_something(data)
            return
        if ch.lower() == 'sup':   #all walker cmds = rst-reset,wlk,sup-stepup,tlt-torso,nck-[3],abt
            return
        if ch.lower() == 'tlt':   #all walker cmds = rst-reset,wlk,sup-stepup,tlt-torso,nck-[3],abt
            return
        if ch.lower() == 'nck':   #all walker cmds = rst-reset,wlk,sup-stepup,tlt-torso,nck-[3],abt
	    print("nck: ",data.R)
            return
        if ch.lower() == 'abt':   #all walker cmds = rst-reset,wlk,sup-stepup,tlt-torso,nck-[3],abt
            return

    def process_something(self,data):
        R = data.R
        th = data.t
        #self.loginfo( "Something %s, %6.3f, %d, %d" % (data.key, data.val, data.row, data.col))
        self.loginfo( "Walk R=%6.3f th=%6.3f len=%6.3f" % (R, th, R*th))
        foot = self.list.step(R,th)
        self.process_foot(foot)
        
    def getRth(self,data):
        return (3.0,0.07)

    # Creates Foot with the current position and orientation of the foot.
    def getROSloc(self, foot_side):
        lffn = self.LEFT_FOOT_FRAME_NAME
	rffn = self.RIGHT_FOOT_FRAME_NAME
        foot_frame = lffn if foot_side == Foot.LEFT else rffn

        footWorld = self.tfBuffer.lookup_transform('world', foot_frame, rospy.Time(),
		rospy.Duration(1.0))
        rot = footWorld.transform.rotation
        loc = footWorld.transform.translation

    	return loc,rot

    # Creates Foot with the current position and orientation of the foot.
    def createFoot(self, side):
        lffn = self.LEFT_FOOT_FRAME_NAME
	rffn = self.RIGHT_FOOT_FRAME_NAME
        foot_frame = lffn if side == Foot.LEFT else rffn

        ROSleft = FootstepDataRosMessage.LEFT
        ROSright = FootstepDataRosMessage.RIGHT
        step_side = ROSleft if side==Foot.LEFT else ROSright
        #fsmsg = self.createFootStepInPlace(step_side)
        #rot = fsmsg.orientation
        #loc = fsmsg.location

        #footWorld = self.tfBuffer.lookup_transform('world', foot_frame, rospy.Time(),
	#	rospy.Duration(1.0))
        #rot = footWorld.transform.rotation
        #loc = footWorld.transform.translation
        loc,rot = self.getROSloc(side)


        euler = euler_from_quaternion([rot.x,rot.y,rot.z,rot.w])
        self.recentz = loc.z
    	return Foot([loc.x,loc.y,euler[2]],side)

    def process_foot(self, foot):
        msg = self.getFootFootstepMsg(foot)
        #self.footstep_publisher.publish(msg)
        res = self.execute_footsteps(msg)
        #if res:
        #    self.loginfo('done walking')
        #    return
        #self.loginfo('failed to walk, aborting trajectory')

    def getFootFootstepMsg(self, foot):
        #msg = self.getEmptyFootsetListMsg()
        msg = FootstepDataListRosMessage()
        msg.default_transfer_time = 0.3 #was transfer_time 1.5
        msg.default_swing_time = 0.7 #was swing_time 1.5
        msg.execution_mode = 0
        msg.unique_id = -1
        ROSleft = FootstepDataRosMessage.LEFT
        ROSright = FootstepDataRosMessage.RIGHT
        footstep = FootstepDataRosMessage()
        step_side = ROSleft if foot.side==Foot.LEFT else ROSright
        footstep.robot_side = step_side
        #loc,rot = getROSloc(side)
        rot = quaternion_from_euler(0,0,foot.t)
	footstep.orientation.x = rot[0]
	footstep.orientation.y = rot[1]
	footstep.orientation.z = rot[2]
	footstep.orientation.w = rot[3]
        footstep.location.x = foot.x
        footstep.location.y = foot.y
        footstep.location.z = self.recentz

        msg.footstep_data_list.append(footstep)

        return msg

    def process_walking_command(self, binding, ch):
        is_lower_case = ch == ch.lower()
        # ROT_STEP = 90*3.14159/180.
        direction = 1.0 if is_lower_case else -1.0
        self.footstep_count = 0
        if binding['action'] == 'translate':
            self.loginfo('Walking ' + ('forward' if is_lower_case else 'backward'))
            self.translate([self.TRANS_STEP * direction, 0.0, 0.0])
        elif binding['action'] == 'rotate':
            self.loginfo('Rotating ' + ('counter-clockwise' if is_lower_case else 'clockwise'))
            self.rotate(self.ROT_STEP * direction)

    def createRotationFootStepList(self, yaw):
        left_footstep = FootstepDataRosMessage()
        left_footstep.robot_side = FootstepDataRosMessage.LEFT
        right_footstep = FootstepDataRosMessage()
        right_footstep.robot_side = FootstepDataRosMessage.RIGHT

        left_foot_world = self.tfBuffer.lookup_transform(
            'world', self.LEFT_FOOT_FRAME_NAME, rospy.Time())
        right_foot_world = self.tfBuffer.lookup_transform(
            'world', self.RIGHT_FOOT_FRAME_NAME, rospy.Time())
        intermediate_transform = Transform()
        # create a virtual fram between the feet, this will be the center of the rotation
        intermediate_transform.translation.x = (
            left_foot_world.transform.translation.x + right_foot_world.transform.translation.x)/2.
        intermediate_transform.translation.y = (
            left_foot_world.transform.translation.y + right_foot_world.transform.translation.y)/2.
        intermediate_transform.translation.z = (
            left_foot_world.transform.translation.z + right_foot_world.transform.translation.z)/2.
        # here we assume that feet have the same orientation so we can pick arbitrary left or right
        intermediate_transform.rotation = left_foot_world.transform.rotation

        left_footstep.location = left_foot_world.transform.translation
        right_footstep.location = right_foot_world.transform.translation

        # define the turning radius
        radius = sqrt(
            (
                right_foot_world.transform.translation.x -
                left_foot_world.transform.translation.x
            )**2 + (
                right_foot_world.transform.translation.y -
                left_foot_world.transform.translation.y
            )**2) / 2.

        left_offset = [-radius*sin(yaw), radius*(1-cos(yaw)), 0]
        right_offset = [radius*sin(yaw), -radius*(1-cos(yaw)), 0]
        intermediate_euler = euler_from_quaternion([
            intermediate_transform.rotation.x,
            intermediate_transform.rotation.y,
            intermediate_transform.rotation.z,
            intermediate_transform.rotation.w])
        resulting_quat = quaternion_from_euler(
            intermediate_euler[0], intermediate_euler[1],
            intermediate_euler[2] + yaw)

        rot = quaternion_matrix([
            resulting_quat[0], resulting_quat[1], resulting_quat[2], resulting_quat[3]])
        left_transformedOffset = np.dot(rot[0:3, 0:3], left_offset)
        right_transformedOffset = np.dot(rot[0:3, 0:3], right_offset)
        quat_final = Quaternion(
            resulting_quat[0], resulting_quat[1], resulting_quat[2], resulting_quat[3])

        left_footstep.location.x += left_transformedOffset[0]
        left_footstep.location.y += left_transformedOffset[1]
        left_footstep.location.z += left_transformedOffset[2]
        left_footstep.orientation = quat_final

        right_footstep.location.x += right_transformedOffset[0]
        right_footstep.location.y += right_transformedOffset[1]
        right_footstep.location.z += right_transformedOffset[2]
        right_footstep.orientation = quat_final

        if yaw > 0:
            return [left_footstep, right_footstep]
        else:
            return [right_footstep, left_footstep]

    # Creates footstep with the current position and orientation of the foot.
    def createFootStepInPlace(self, step_side):
        footstep = FootstepDataRosMessage()
        footstep.robot_side = step_side

        if step_side == FootstepDataRosMessage.LEFT:
            foot_frame = self.LEFT_FOOT_FRAME_NAME
        else:
            foot_frame = self.RIGHT_FOOT_FRAME_NAME

        footWorld = self.tfBuffer.lookup_transform('world', foot_frame, rospy.Time())
        footstep.orientation = footWorld.transform.rotation
        footstep.location = footWorld.transform.translation

        return footstep

    # Creates footstep offset from the current foot position. The offset is in foot frame.
    def createTranslationFootStepOffset(self, step_side, offset):
        footstep = self.createFootStepInPlace(step_side)

        # transform the offset to world frame
        quat = footstep.orientation
        rot = quaternion_matrix([quat.x, quat.y, quat.z, quat.w])
        transformedOffset = np.dot(rot[0:3, 0:3], offset)

        footstep.location.x += transformedOffset[0]
        footstep.location.y += transformedOffset[1]
        footstep.location.z += transformedOffset[2]

        return footstep

    def getEmptyFootsetListMsg(self):
        msg = FootstepDataListRosMessage()
        msg.default_transfer_time = 0.3 #was transfer_time 1.5
        msg.default_swing_time = 0.7 #was swing_time 1.5
        #msg.transfer_time = 0.9 #was 1.5
        #msg.swing_time = 0.7 #was 1.5
        msg.execution_mode = 0
        msg.unique_id = -1
        return msg

    def getTranslationFootstepMsg(self, offset):
        msg = self.getEmptyFootsetListMsg()
        msg.footstep_data_list.append(self.createTranslationFootStepOffset(
            FootstepDataRosMessage.LEFT, offset))
        msg.footstep_data_list.append(self.createTranslationFootStepOffset(
            FootstepDataRosMessage.RIGHT, offset))
        return msg

    def getRotationFooststepMsg(self, yaw):
        msg = self.getEmptyFootsetListMsg()
        footsteps = self.createRotationFootStepList(yaw)
        for footstep in footsteps:
            msg.footstep_data_list.append(footstep)

        return msg

    def execute_footsteps(self, msg):
        self.footstep_count = 0
        self.footstep_publisher.publish(msg)
        number_of_footsteps = len(msg.footstep_data_list)
        max_iterations = 100
        count = 0
        while count < max_iterations:
            self.rate.sleep()
            count += 1
            if self.footstep_count == number_of_footsteps:
                return True
                break
        msg = AbortWalkingRosMessage()
        msg.unique_id = -1
        self.abort_walking_publisher.publish(msg)
        return False

    def translate(self, offset):
        msg = self.getTranslationFootstepMsg(offset)
        res = self.execute_footsteps(msg)
        if res:
            self.loginfo('done walking')
            return
        self.loginfo('failed to walk, aborting trajectory')

    def rotate(self, yaw):
        # space feet further apart if not spaced enough for safe rotation
        self.set_init_pose()
        msg = self.getRotationFooststepMsg(yaw)
        res = self.execute_footsteps(msg)
        if res:
            self.loginfo('done rotating')
            return
        self.loginfo('failed to rotate, aborting trajectory')

    def set_init_pose(self):
        left_foot_world = self.tfBuffer.lookup_transform(
            'world', self.LEFT_FOOT_FRAME_NAME, rospy.Time())
        right_foot_world = self.tfBuffer.lookup_transform(
            'world', self.RIGHT_FOOT_FRAME_NAME, rospy.Time())
        intermediate_transform = Transform()
        distance_between_feet = sqrt(
            (
                right_foot_world.transform.translation.x -
                left_foot_world.transform.translation.x
            )**2 + (
                right_foot_world.transform.translation.y -
                left_foot_world.transform.translation.y
            )**2)
        if distance_between_feet <= 0.2:
            self.loginfo('moving feet further apart\n')
            msg = self.getEmptyFootsetListMsg()
            msg.footstep_data_list.append(self.createTranslationFootStepOffset(
                FootstepDataRosMessage.LEFT, [0.0, 0.05, 0.0]))
            msg.footstep_data_list.append(self.createTranslationFootStepOffset(
                FootstepDataRosMessage.RIGHT, [0.0, -0.05, 0.0]))
            self.execute_footsteps(msg)
            self.loginfo('done moving feet further apart\n')




class KeyboardTeleop(object):

    ARM_BINDINGS = OrderedDict([
        ('q', {'joint_index': 0, 'side': 'left', 'min': -2.85, 'max': 2.0}),  # leftShoulderPitch
        ('w', {'joint_index': 1, 'side': 'left', 'min': -1.519, 'max': 1.266,
               'invert': True}),  # leftShoulderRoll
        ('e', {'joint_index': 2, 'side': 'left', 'min': -3.1, 'max': 2.18}),  # leftShoulderYaw
        ('r', {'joint_index': 3, 'side': 'left', 'min': -2.174, 'max': 0.12,
               'invert': True}),  # leftElbowPitch
        ('a', {'joint_index': 4, 'side': 'left', 'min': -2.019, 'max': 3.14}),  # leftForearmYaw
        ('s', {'joint_index': 5, 'side': 'left', 'min': -0.62, 'max': 0.625,
               'invert': True}),  # leftWristRoll
        ('d', {'joint_index': 6, 'side': 'left', 'min': -0.36, 'max': 0.49,
               'invert': True}),  # leftWristPitch
        ('t', {'joint_index': 'reset', 'side': 'left'}),

        ('u', {'joint_index': 0, 'side': 'right', 'min': -2.85, 'max': 2.0}),  # rightShoulderPitch
        ('i', {'joint_index': 1, 'side': 'right', 'min': -1.266, 'max': 1.519}),  # rightShoulderRoll
        ('o', {'joint_index': 2, 'side': 'right', 'min': -3.1, 'max': 2.18}),  # rightShoulderYaw
        ('p', {'joint_index': 3, 'side': 'right', 'min': -0.12, 'max': 2.174}),  # rightElbowPitch
        ('j', {'joint_index': 4, 'side': 'right', 'min': -2.019, 'max': 3.14}),  # rightForearmYaw
        ('k', {'joint_index': 5, 'side': 'right', 'min': -0.625, 'max': 0.62}),  # rightWristRoll
        ('l', {'joint_index': 6, 'side': 'right', 'min': -0.49, 'max': 0.36}),  # rightWristPitch
        ('y', {'joint_index': 'reset', 'side': 'right'}),
    ])

    HEAD_BINDINGS = OrderedDict([
        ('x', {'joint_index': 0, 'min': -0.5, 'max': 0.5}),
        ('c', {'joint_index': 1, 'min': -0.5, 'max': 0.5}),
        ('v', {'joint_index': 2, 'min': -1.0, 'max': 1.0}),
        ('g', {'joint_index': 'reset'}),
    ])

    NECK_BINDINGS = OrderedDict([
        ('b', {'joint_index': 0, 'min': 0.0, 'max': 0.5}),  # lowerNeckPitch? 'min': 0, 'max': 1.162
        ('n', {'joint_index': 1, 'min': -1.0, 'max': 1.0}),  # neckYaw? 'min': -1.047, 'max': 1.047
        ('m', {'joint_index': 2, 'min': -0.5, 'max': 0.0}),  # upperNeckPitch? 'min': -0.872, 'max': 0
        ('h', {'joint_index': 'reset'}),
    ])

    WALKING_BINDINGS = OrderedDict([
        ('f', {'action': 'translate'}),
        ('z', {'action': 'rotate'}),
    ])

    def __init__(self,verbose=3):
        self.joint_values = {
            'left arm': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            'right arm': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            'head': [0.0, 0.0, 0.0],
            'neck': [0.0, 0.0, 0.0],
        }
        # ensure no characters are bound repeatedly
        bindings = [self.ARM_BINDINGS, self.HEAD_BINDINGS, self.NECK_BINDINGS]
        keys = set([])
        [keys.update(b.keys()) for b in bindings]
        assert len(keys) == sum([len(b) for b in bindings]), 'Duplicate binding'
        self.v = verbose
        self.walker=Walker(verbose=self.v,logger=self.loginfo)
        #self.walker.init()

    def run(self):
        try:
            self.init()
	    self.walker.init()
            self.print_usage()
            while not rospy.is_shutdown():
                ch = self.get_key()
                self.process_key(ch)
        finally:
            self.fini()

    def init(self):
        # save terminal settings
        self.settings = termios.tcgetattr(sys.stdin)

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

        # make sure the simulation is running otherwise wait
        self.rate = rospy.Rate(10)  # 10hz
        publishers = [self.arm_publisher, self.neck_publisher, self.head_publisher]
        if any([p.get_num_connections() == 0 for p in publishers]):
            rospy.loginfo('waiting for subscriber...')
            while any([p.get_num_connections() == 0 for p in publishers]):
                self.rate.sleep()

    def fini(self):
        self.arm_publisher = None

        # restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def print_usage(self):
        def get_bound_char(bindings, x):
            return x.upper() if bindings[x].get('invert', False) else x

        msg_args = []

        # left arm - first row
        msg_args.append(', '.join([
            get_bound_char(self.ARM_BINDINGS, x)
            for x in self.ARM_BINDINGS.keys()[0:4]]))
        # left arm - second row
        msg_args.append(', '.join([
            get_bound_char(self.ARM_BINDINGS, x)
            for x in self.ARM_BINDINGS.keys()[4:7]]))
        # left arm - reset
        msg_args.append(' or '.join([
            self.ARM_BINDINGS.keys()[7], self.ARM_BINDINGS.keys()[7].upper()]))

        # right arm - first row
        msg_args.append(', '.join([
            get_bound_char(self.ARM_BINDINGS, x)
            for x in self.ARM_BINDINGS.keys()[8:12]]))
        # right arm - second row
        msg_args.append(', '.join([
            get_bound_char(self.ARM_BINDINGS, x)
            for x in self.ARM_BINDINGS.keys()[12:15]]))
        # right arm - reset
        msg_args.append(' or '.join([
            self.ARM_BINDINGS.keys()[15], self.ARM_BINDINGS.keys()[15].upper()]))

        # head - roll, pitch, yaw
        msg_args += [
            get_bound_char(self.HEAD_BINDINGS, x)
            for x in self.HEAD_BINDINGS.keys()[0:3]]
        # head - reset
        msg_args.append(' or '.join([
            self.HEAD_BINDINGS.keys()[3], self.HEAD_BINDINGS.keys()[3].upper()]))

        # neck
        msg_args.append(', '.join([
            get_bound_char(self.NECK_BINDINGS, x)
            for x in self.NECK_BINDINGS.keys()[0:3]]))
        # neck - reset
        msg_args.append(' or '.join([
            self.NECK_BINDINGS.keys()[3], self.NECK_BINDINGS.keys()[3].upper()]))

        # walking
        [msg_args.append('{}'.format(x)) for x in self.WALKING_BINDINGS.keys()]

        print(msg_args)

        msg = """
            Keyboard Teleop for Space Robotics Challenge 0.1.0
            Copyright (C) 2017 Open Source Robotics Foundation
            Released under the Apache 2 License
            --------------------------------------------------
            Left arm joints: {}
                             {}
                Reset: {}

            Right arm joints: {}
                              {}
                Reset: {}

            Head angles:
                Roll: {}
                Pitch: {}
                Yaw: {}
                Reset all head angles: {}

            Neck joints: {}
                Reset all neck joints: {}

            The shown characters turn the joints in positive direction.
            The opposite case turns the joints in negative direction.

            Walking controls:
                Walk: {} lowercase meaning forward and uppercase backwards
                Rotating on the spot: {} lowercase is counterclockwise, uppercase is clockwise
            ?: Print this menu
            <TAB>: Print all current joint values
            <ESC>: Quit
            """.format(*msg_args)
        self.loginfo(msg)

    def get_key(self):
        """Get input from the terminal."""
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        return key

    def process_key(self, ch):
        """Process key event."""
        if ch == '?':
            self.print_usage()
            return

        if ord(ch) == 9:  # TAB key
            for label in sorted(self.joint_values.keys()):
                self.loginfo(
                    'Current joint values for the %s: %s' %
                    (label, self.joint_values[label]))
            return

        if ord(ch) == 27:  # ESCAPE key
            self.loginfo('Quitting')
            rospy.signal_shutdown('Shutdown')
            return

        if ch.lower() in self.ARM_BINDINGS:
            self.process_arm_command(self.ARM_BINDINGS[ch.lower()], ch)
            return

        if ch.lower() in self.HEAD_BINDINGS:
            self.process_head_command(self.HEAD_BINDINGS[ch.lower()], ch)
            return

        if ch.lower() in self.NECK_BINDINGS:
            self.process_neck_command(self.NECK_BINDINGS[ch.lower()], ch)
            return

        if ch.lower() in self.WALKING_BINDINGS:
            self.walker.process_keys([Astep(ch,0.0)])
            #self.process_walking_command(self.WALKING_BINDINGS[ch.lower()], ch)
            return

    def process_arm_command(self, binding, ch):
        msg = ArmTrajectoryRosMessage()
        msg.unique_id = -1

        side = binding['side']
        if side == 'left':
            msg.robot_side = ArmTrajectoryRosMessage.LEFT
        elif side == 'right':
            msg.robot_side = ArmTrajectoryRosMessage.RIGHT
        else:
            assert False, "Unknown arm side '%s'" % side

        self._update_joint_values('%s arm' % side, binding, ch)
        self._append_trajectory_point_1d(
            msg, 1.0, self.joint_values['%s arm' % side])

        self.arm_publisher.publish(msg)

    def process_head_command(self, binding, ch):
        msg = HeadTrajectoryRosMessage()
        msg.unique_id = -1
        self._update_joint_values('head', binding, ch)
        self._append_trajectory_point_so3(
            msg, 1.0, self.joint_values['head'])
        self.head_publisher.publish(msg)

    def process_neck_command(self, binding, ch):
        msg = NeckTrajectoryRosMessage()
        msg.unique_id = -1
        self._update_joint_values('neck', binding, ch)
        self._append_trajectory_point_1d(
            msg, 1.0, self.joint_values['neck'])
        self.neck_publisher.publish(msg)


    def _update_joint_values(self, label, binding, ch):
        joint_index = binding['joint_index']
        if joint_index == 'reset':
            # reset all joints to zero
            self.loginfo('Reset %s' % label)
            self.joint_values[label] = [0.0] * len(self.joint_values[label])

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
                'Move %s joint #%d %s to %.1f: [%s]' %
                (label, joint_index, '++' if (is_lower_case != binding.get('invert', False)) else '--',
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
        """Log info message while terminal is in funky mode."""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        rospy.loginfo(msg)
        tty.setraw(sys.stdin.fileno())


if __name__ == '__main__':
    rospy.init_node('walker')
    teleop = KeyboardTeleop()
    teleop.run()

'''
As of 4/14/17??
ihmc_msgs/FootstepDataListRosMessage
uint8 OVERRIDE=0
uint8 QUEUE=1
ihmc_msgs/FootstepDataRosMessage[] footstep_data_list
  uint8 LEFT=0
  uint8 RIGHT=1
  uint8 AT_ANKLE_FRAME=0
  uint8 AT_SOLE_FRAME=1
  uint8 DEFAULT=0
  uint8 OBSTACLE_CLEARANCE=1
  uint8 CUSTOM=2
  uint8 origin
  uint8 robot_side
  geometry_msgs/Vector3 location
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion orientation
    float64 x
    float64 y
    float64 z
    float64 w
  ihmc_msgs/Point2dRosMessage[] predicted_contact_points
    float64 x
    float64 y
  uint8 trajectory_type
  geometry_msgs/Vector3[] trajectory_waypoints
    float64 x
    float64 y
    float64 z
  float64 swing_height
  bool has_timings
  float64 swing_time
  float64 transfer_time
  bool has_absolute_time
  float64 swing_start_time
  int64 unique_id
uint8 execution_mode
float64 default_swing_time
float64 default_transfer_time
float64 final_transfer_time
int64 unique_id

'''
