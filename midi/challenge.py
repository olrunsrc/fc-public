#!/usr/bin/env python
from __future__ import print_function

import rospy
import tf
import tf2_ros
import numpy as np
import time

from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion

from srcstate import SrcState
from challengepages import Pages, Qitem
from websocksrv import WSS, ChallengeProtocolA
import thread
import threading
import Queue
import os,sys,logging
import subprocess as prc
import mmap
import urlparse
import posix_ipc
from ipcomm import IPComm 

class Challenge(object):

    ''' Class to monitor and control challenge specific messages and services
    '''
    PORT = '8083'

    def __init__(self):
        ''' connect to ros and set up web page and web sockets '''
	self.ipcWS = IPComm("chalmap")
	self.ipcHT = IPComm("htmlmap")
	self.state = SrcState()
        print("init")

    def init(self):
        ''' connect to ros and set up web page '''
        self.HTdata = 0
	self.web = Pages(self)
        port = Challenge.PORT
        rospy.loginfo("Challenge listening on port %s" % port)
	os.environ['PORT'] = port
	self.state.init()

    def initWS(self,ip):
        ''' set up web sockets '''
        cmd = "python -u websocksrv.py %s" % ip
	print("Starting %s"%cmd)
  	self.wsproc = prc.Popen(cmd, shell=True )


    def finish(self):
	self.ipcWS = None
	self.ipcHT = None
	self.state.fini()
	self.web.finish()
        self.wsproc.terminate()

    def loop(self):
        #print("top of loop")
        self.HTdata += 1
        done = False
        with self.ipcHT.sem:
          #print("loop got HTsem")
          if not self.ipcHT.empty():
            print("queue has %d items"%self.ipcHT.queue.current_messages)
            item,_ = self.ipcHT.queue.receive()
            self.HTdata += 1
            print(item)
	    try:
		print( "Queued data is %s" % item )
		request = dict(urlparse.parse_qsl(item))    
		msg = str(request.get('msg','NOOP'))
		print( "msg: %s" % msg )
		if msg=='exit':
		    print("Prepare to exit")
		    done = True
		if msg.lower() in SrcState.CMDS:
		    self.state.loop(msg)
		self.ipcHT.mmap.seek(0)
		self.ipcHT.mmap.write(self.HTdata)
		self.ipcHT.mmap.write("\n")
	    except Exception as e:
		print(e)
		self.ipcHT.mmap.seek(0)
		self.ipcHT.mmap.write(str(e))
		self.ipcHT.mmap.write("\n")
        #print("top of WSsem loop")
        with self.ipcWS.sem:
            #print("loop got WSsem")
	    try:
		_,data = self.state.loop(None)
		#print("WSsem loop data is %s"%data)
		self.ipcWS.mmap.seek(0)
		self.ipcWS.mmap.write(data)
		self.ipcWS.mmap.write("\n")
	    except Exception as e:
		print(e)
        return done

    def run(self):
        try:
            rate = rospy.Rate(10) # 10hz
	    thread.start_new_thread(self.web.run, ())
            while not rospy.is_shutdown():
                done = self.loop()
                if done:
                    rospy.signal_shutdown("Challenge Done")
                rate.sleep()
	except rospy.ROSInterruptException:
 	    pass
        finally:
            rospy.loginfo("Challenge Exiting")
            self.finish()

if __name__ == '__main__':
    logging.basicConfig(
        level=logging.INFO,
        format='%(threadName)10s %(name)18s: %(message)s',
        stream=sys.stderr,
    )
    rospy.init_node('Challenge')
    teleop = Challenge()
    teleop.initWS(sys.argv[1])
    teleop.init()
    teleop.run()


#SRCSIM messages and commands
'''
rosservice call /srcsim/finals/start_task 1 1
/srcsim/finals/task
/task1/checkpoint2/satellite
/task3/checkpoint5/leak

task: 3
current_checkpoint: 3
checkpoints_completion:
  -
    secs: 40
    nsecs: 332000000
  -
    secs: 40
    nsecs: 756000000
start_time:
  secs: 35
  nsecs: 236000000
elapsed_time:
  secs: 5
  nsecs: 520000000
timed_out: False
finished: False

target_pitch: -0.2
target_yaw: 1.0
current_pitch: -0.0306981597753
current_yaw: 0.238853778623
pitch_correct_now: False
yaw_correct_now: False
pitch_completed: False
yaw_completed: False



value: 0.01
'''
