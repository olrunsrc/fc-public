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

Kib = 1024
Mib = Kib*Kib

class Challenge(object):

    ''' Class to monitor and control challenge specific messages and services
    '''
    PORT = '8083'

    def __init__(self):
        ''' connect to ros and set up web page and web sockets '''
	#self.memfile = open("data/chalmap","r+b")
	#self.memmap = mmap.mmap(self.memfile.fileno(),0)
        print("init")
	memory = posix_ipc.SharedMemory("chalmap", posix_ipc.O_CREAT, size=8*Kib)
        print("init")
	self.memmap = mmap.mmap(memory.fd, memory.size)
        print("init")
	memory.close_fd()
        print("init")
	self.state = SrcState()
        print("init")

    def init(self):
        ''' connect to ros and set up web page '''
        self.queue = Queue.Queue()
        self.data = 0
	self.web = Pages(self)
        port = Challenge.PORT
        rospy.loginfo("Challenge listening on port %s" % port)
	os.environ['PORT'] = port
	self.state.init()

    def initWS(self):
        ''' set up web sockets '''
  	self.wsproc = prc.Popen('python websocksrv.py', shell=True )

    def finish(self):
	self.memmap.flush()
	self.memmap.close()
	#self.memfile.close()
	self.state.fini()
	self.web.finish()
        self.wsproc.terminate()


    def loop(self):
        self.data += 1
        done = False
        if not self.queue.empty():
            item = self.queue.get()
            self.data += 1
	    try:
		print( "Queued data is %s" % item.req )
		request = dict(urlparse.parse_qsl(item.req))    
		msg = str(request.get('msg','NOOP'))
		print( "msg: %s" % msg )
		if msg=='exit':
		    print("Prepare to exit")
		    #self.finish()
		    done = True
		if msg.lower() in SrcState.CMDS:
		    self.state.loop(msg)
		item.resp = self.data
	    except Exception as e:
		print(e)
		item.resp = e

            item.evt.set()
            #item.evt.clear()
            self.queue.task_done()
	try:
		_,data = self.state.loop(None)
		self.memmap.seek(0)
		self.memmap.write(data)
		self.memmap.write("\n")
	except Exception as e:
		print(e)
        return done

    def run(self):
        try:
            rate = rospy.Rate(10) # 10hz
	    thread.start_new_thread(self.web.run, ())
	    #self.p.start()
	    #thread.start_new_thread(self.ws.run, ())
            #self.ws.run()
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
    teleop.initWS()
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
