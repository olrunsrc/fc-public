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
import json

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

from srcsim.srv import StartTask
from srcsim.msg import Task
from srcsim.msg import Satellite
from srcsim.msg import Leak  #value f64
#from srcsim.msg import Harness #status uint8
#from srcsim.msg import Score 

LEFT = 0
RIGHT = 1

'''
float64 target_pitch
float64 target_yaw
float64 current_pitch
float64 current_yaw
bool pitch_correct_now
bool yaw_correct_now
bool pitch_completed
bool yaw_completed

uint32 task
uint32 current_checkpoint
time[] checkpoints_completion
time start_time
time elapsed_time
bool timed_out
bool finished

duration[] checkpoint_durations
duration[] checkpoint_penalties
uint8 score
duration total_completion_time

'''
class JEnc(json.JSONEncoder):
    def default(self, o):
	if isinstance(o, float):
	    return "%6.3f"%o
	return o.__dict__

class TaskState:
    def __init__(self,n):
	self.task = float(n)
	self.curr = 0.0
        self.comp = [ -1.0 for i in range(8) ]
	self.start = -1.0
	self.elapsed = 0.0
	self.timedout = 0.0
	self.finished = 0.0

    def update(self, msg):
	if float(msg.task)==self.task:
	    self.curr = float(msg.current_checkpoint)
	    for i,v in enumerate( msg.checkpoint_durations ):
		self.comp[i] = v.to_sec()
	    self.start = msg.start_time.to_sec()
	    self.elapsed = msg.elapsed_time.to_sec()
	    self.timedout = float(1 if msg.timed_out else 0)
	    self.finished = float(1 if msg.finished else 0)

class SatState:
    def __init__(self, dir):
	self.dir = float(dir) #0 pitch, 1 yaw
	self.tgt = 0.0
	self.cur = 0.0
	self.ok  = 0.0
	self.done= 0.0

    def update(self, msg):
	if self.dir==0.0:
	    self.tgt = float(msg.target_pitch)
	    self.cur = float(msg.current_pitch)
	    self.ok  = float(msg.pitch_correct_now)
	    self.done= float(msg.pitch_completed)
	else:
	    self.tgt = float(msg.target_yaw)
	    self.cur = float(msg.current_yaw)
	    self.ok  = float(msg.yaw_correct_now)
	    self.done= float(msg.yaw_completed)

class ScoreState:
    def __init__(self):
	self.score = 0.0
	self.total = 0.0
        self.dur = [ -1.0 for i in range(18) ]
        self.pen = [ -1.0 for i in range(18) ]

    def update(self, msg):
	    self.score = float(msg.current_checkpoint)
	    self.total = float(msg.current_checkpoint)
	    for i,v in enumerate( msg.checkpoint_durations ):
		self.dur[i] = v.to_sec()
	    for i,v in enumerate( msg.checkpoint_penalties ):
		self.pen[i] = v.to_sec()


class ChallengeState:
    def __init__(self):
	self.current = 0.0
	self.task1 = TaskState(1)
	self.task2 = TaskState(2)
	self.task3 = TaskState(3)
	self.satpitch = SatState(0)
	self.satyaw = SatState(1)
	self.leak = 0.0
	self.harness = 0.0
        self.clock = 0.0
	self.score = ScoreState()

    def update(self, msg):
	task = 0
	if msg.__class__.__name__=='Task':
	    task = msg.task
	if task==1:
	    self.current = 1.0
	    self.task1.update(msg)
	if task==2:
	    self.current = 2.0
	    self.task2.update(msg)
	if task==3:
	    self.current = 3.0
	    self.task3.update(msg)
	if msg.__class__.__name__=='Satellite':
	    self.satpitch.update(msg)
	    self.satyaw.update(msg)
	if msg.__class__.__name__=='Leak':
	    self.leak = float(msg.value)
	if msg.__class__.__name__=='Score':
	    self.score.update(msg)
	if msg.__class__.__name__=='Harness':
	    self.harness = float(msg.status)

class SrcState:
    INTERVAL = 0.05
    CMDS = ['a','b','c','2','3','4','5','6','7','8']
    def __init__(self):
	self.state = ChallengeState()
	self.nexttime = 0.0
	self.nextsattime = 0.0
	self.nextleaktime = 0.0
	self.initted1 = False
	self.initted3 = False

    def init(self):
	ROBOT_NAME = rospy.get_param('/ihmc_ros/robot_name')
	fss_name = "/ihmc_ros/{0}/output/footstep_status".format(ROBOT_NAME)
	pose_name = "/ihmc_ros/{0}/output/robot_pose".format(ROBOT_NAME)

	fssSubscriber = rospy.Subscriber( fss_name, FootstepStatusRosMessage, self.rcvdfss)
	poseSubscriber = rospy.Subscriber( pose_name, Odometry, self.rcvdpose )

	tasks_nm = "/srcsim/finals/task"
	taskSubscriber = rospy.Subscriber( tasks_nm, Task, self.rcvdmsg )

        taskpx_nm = "/srcsim/finals/start_task"
	self.taskProxy = rospy.ServiceProxy(taskpx_nm, StartTask)

	self.tfBuffer = tf2_ros.Buffer()
	tfListener = tf2_ros.TransformListener(self.tfBuffer)

	self.stepCounter = 0
	self.robotpose = None

	time.sleep(1)

    def rosTask(self,t,c):
        #print("Starting task (%d,%d)" % (t,c))
        ret = self.taskProxy(t,c)
        print("Started task (%d,%d) returned %s" % (t,c,ret))

    def init1(self):
        sats_nm = "/task1/checkpoint2/satellite"
	satSubscriber = rospy.Subscriber( sats_nm, Satellite, self.rcvdsat )
	self.initted1 = True

    def init3(self):
	leaks_nm = "/task3/checkpoint5/leak"
	leakSubscriber = rospy.Subscriber( leaks_nm, Leak, self.rcvdleak )
	self.initted3 = True

    def fini(self):
	pass

    def rcvdfss(self,msg):
	if msg.status == 1:
	    self.stepCounter += 1

    def rcvdpose(self,msg):
	self.robotpose = msg

    def rcvdmsg(self,msg):
	now = rospy.Time.now().to_sec()
	if now > self.nexttime:
	    self.nexttime = now + SrcState.INTERVAL
	    self.state.update(msg)
	    if self.state.current==1.0 and not self.initted1:
		self.init1()
	    if self.state.current==3.0 and not self.initted3:
		self.init3()

    def rcvdsat(self,msg):
	now = rospy.Time.now().to_sec()
	if now > self.nextsattime:
	    self.nextsattime = now + SrcState.INTERVAL
	    self.state.update(msg)

    def rcvdleak(self,msg):
	now = rospy.Time.now().to_sec()
	if now > self.nextleaktime:
	    self.nextleaktime = now + SrcState.INTERVAL
	    self.state.update(msg)

    def loop(self,action):
	done = action and ord(action)==27
	data = self.state
        if action and action.lower()=='a':
	    self.taskProxy(1,1)
	    self.state.current = max(1.0,self.state.current)
        if action and action.lower()=='b':
	    self.taskProxy(2,1)
	    self.state.current = max(2.0,self.state.current)
        if action and action.lower()=='c':
	    self.taskProxy(3,1)
	    self.state.current = max(3.0,self.state.current)
        if action and ord(action)>=ord('2') and ord(action)<=ord('8'):  
	    self.taskProxy(int(self.state.current),ord(action)-ord('0'))
	return done, json.dumps(data,cls=JEnc)

if __name__ == '__main__':
    rospy.init_node('Challenge_State')
    kb = Keyboard()
    mon = SrcState()
    mon.init()
    kb.init()
    try:
	rate = rospy.Rate(1) # 1 hz 
	while not rospy.is_shutdown():
	    ch = kb.get_key()
	    done, data = mon.loop(ch)
	    if done:
		rospy.signal_shutdown("Done")
	    kb.loginfo(data)
	    rate.sleep()
    except rospy.ROSInterruptException:
	pass
    finally:
	kb.fini()
        mon.fini()
	rospy.loginfo("Exiting")

  #lfootWorld = self.tfBuffer.lookup_transform('world', LEFT_FOOT_FRAME_NAME, rospy.Time())


