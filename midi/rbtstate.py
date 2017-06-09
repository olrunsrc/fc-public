#!/usr/bin/env python
from __future__ import print_function

from keyboard import Keyboard
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

from sensor_msgs.msg import JointState
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
from tf.transformations import euler_from_quaternion

from gazebo_msgs.srv import GetLinkState

LEFT = 0
RIGHT = 1

class RbtSub:
    def __init__(self,parent):
	self.parent = parent
	self.sub = None
        self.names = ['None']
	self.val = [0]

    def __repr__(self):
        return "!%s ('%s')" % (self.__class__.__name__, "','".join(self.names))

    def hasName(self,name):
	return name in self.names

    def getVal(self,i):
	return self.val[i]

    def subscribe(self, namelist, v=None):
	if not v: v = self.getVal
	idx = [self.names.index(nm) for nm in namelist]
	print("Subscribe ",namelist,"Indexes ",idx)
	#return lambda : [ v[i] for i in idx ] 
	return self.getdataFn(v,idx) 

    def cb(self,msg):
	self.val = msg

    def getdataFn(self,data,idx):
	def fn():
	    if callable(data): 
		return [ data(i) for i in idx ]
	    else:
		return [ data[i] for i in idx ]
	return fn

class RbtHands(RbtSub):
    def __init__(self,parent):
	RbtSub.__init__(self,parent)
	self.names = ['torso->leftPalm','torso->rightPalm','torso->world','world->torso',
		'pelvis->world','leftPalm->world']
	self.val = [[0,0,0,0,0,0,0],[0,0,0,0,0,0,0]]

    def getVal(self,i=0):
	#if i == 0:
	#   v = self.parent.getTf2('torso','leftPalm')
	#else:
	#   v = self.parent.getTf2('torso','rightPalm')
	parts = self.names[i].split('->')
	v = self.parent.getTf2(parts[0],parts[1])
	t = v.transform.translation
	q = v.transform.rotation
	return [t.x,t.y,t.z,q.x,q.y,q.z,q.w]
	
#    def subscribe(self, namelist):
#	idx = [self.names.index(nm) for nm in namelist]
#	return lambda : [ self.getVal(i) for i in idx ]


class RbtJoints(RbtSub):
    def __init__(self,parent):
	RbtSub.__init__(self,parent)
	jss_name = "/joint_states"
	self.sub = rospy.Subscriber( jss_name, JointState, self.cb )
	self.eff = None

    def getEffort(self,i):
	return self.eff[i]

    def subscribe(self, namelist, type='position'):
	if type=='effort':
	    return RbtSub.subscribe(self,namelist,v=self.getEffort)
	if type=='position':
	    return RbtSub.subscribe(self,namelist)

    def cb(self,msg): #header,name,position,velocity,effort
	self.val = msg.position
	self.eff = msg.effort
	if self.names == ['None']:
	    self.names = msg.name

class RbtState:
    def __init__(self):
	self.ROBOT_NAME = rospy.get_param('/ihmc_ros/robot_name')
	self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
	self.providers = [RbtJoints(self),RbtHands(self)]


    def getTf2(self,frfr,frto):
	return self.tfBuffer.lookup_transform(frfr, frto, rospy.Time())

    def fini(self):
	pass

    def subscribe(self, namelist):
	#check for names
	provlist = [] #will be a list of (provider,name)...
	remain = namelist
	for prov in self.providers:
	    remrem = []
	#add names if possible
	    for nm in remain:
		if nm in prov.names: #prov.hasName(nm):
		    provlist.append( (prov,nm) )
		else:
		    remrem.append(nm)
	    remain = remrem
	if remain == []:
	    sublist = []              #will be a list of (provider,[names...])...
	    idx = [0] * len(namelist) #indexes into concatenated provider arrays
	#gather indexes
	    cnt = 0
	    for prov in self.providers:
		pname = []
		for (pr,nm) in provlist:
		    if pr == prov:
			pname.append(nm)
			idx[namelist.index(nm)] = cnt
			cnt += 1
		if pname != []:
		    sublist.append((prov,pname))
	    subscriptions = [ prov.subscribe(pname) for (prov,pname) in sublist ]
	    return self.getSubFn( subscriptions, idx ) 
		#lambda : [ [ sub() for sub in subscriptions ][i] for i in idx ]
	else:
	    print("Names %s not found." % remain)
	    return lambda : [0.0] * len(namelist) 

    def getSubFn(self,subs,idx):
	def subfn():
	    data = []
	    for s in subs:
		print("Data for %s" % idx)
		data.extend( s() )
	    return [ data[i] for i in idx ]
	return subfn


if __name__ == '__main__':
    rospy.init_node('RbtState_test')
    rate = rospy.Rate(3.3) # Hz sim time
    mon = RbtState()
    time.sleep(3)
    lefthand = mon.subscribe(['leftThumbRoll', 'leftThumbPitch1','leftIndexFingerPitch1','leftMiddleFingerPitch1'])
    righthand = mon.subscribe(['rightThumbRoll', 'rightThumbPitch1','rightIndexFingerPitch1','rightMiddleFingerPitch1'])
    leftpalm = mon.subscribe(['torso->leftPalm'])
    three = mon.subscribe(['torsoYaw', 'torsoPitch', 'torsoRoll'])
    five = mon.subscribe(['torsoPitch', 'torso->leftPalm','torso->rightPalm','leftShoulderPitch','rightShoulderPitch'])
    balance = mon.subscribe(['torso->world','leftPalm->world','torso->leftPalm'])
    print(righthand())
    print(five())
    print(three())
    try:
	#print(globals())
	while not rospy.is_shutdown():
	    meas = balance()
	    #print("Left ", map(lambda x: round(x,3), lefthand()))
	    #print("Left ", map(lambda x: round(x,3), leftpalm()[0]))
	    print("Torso", map(lambda x: round(x,3), meas[0]))
	    print("Palm ", map(lambda x: round(x,3), meas[1]))
	    print("Diff ", map(lambda x: round(x,3), meas[2]))
	    #print("Left ", leftpalm() )
	    rate.sleep()
	    #print("Right", map(lambda x: round(x,3), righthand()))
	    #rate.sleep()
    except rospy.ROSInterruptException:
	pass



joints = ['leftAnklePitch', 'leftAnkleRoll', 'leftElbowPitch', 'leftForearmYaw', 'leftHipPitch', 'leftHipRoll', 'leftHipYaw', 'leftIndexFingerPitch1', 'leftIndexFingerPitch2', 'leftIndexFingerPitch3', 'leftKneePitch', 'leftMiddleFingerPitch1', 'leftMiddleFingerPitch2', 'leftMiddleFingerPitch3', 'leftPinkyPitch1', 'leftPinkyPitch2', 'leftPinkyPitch3', 'leftShoulderPitch', 'leftShoulderRoll', 'leftShoulderYaw', 'leftThumbPitch1', 'leftThumbPitch2', 'leftThumbPitch3', 'leftThumbRoll', 'leftWristPitch', 'leftWristRoll', 'lowerNeckPitch', 'neckYaw', 'rightAnklePitch', 'rightAnkleRoll', 'rightElbowPitch', 'rightForearmYaw', 'rightHipPitch', 'rightHipRoll', 'rightHipYaw', 'rightIndexFingerPitch1', 'rightIndexFingerPitch2', 'rightIndexFingerPitch3', 'rightKneePitch', 'rightMiddleFingerPitch1', 'rightMiddleFingerPitch2', 'rightMiddleFingerPitch3', 'rightPinkyPitch1', 'rightPinkyPitch2', 'rightPinkyPitch3', 'rightShoulderPitch', 'rightShoulderRoll', 'rightShoulderYaw', 'rightThumbPitch1', 'rightThumbPitch2', 'rightThumbPitch3', 'rightThumbRoll', 'rightWristPitch', 'rightWristRoll', 'torsoPitch', 'torsoRoll', 'torsoYaw', 'upperNeckPitch']


