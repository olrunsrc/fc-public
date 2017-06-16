#!/usr/bin/env python
from __future__ import print_function

import rospy
import cv2
import time
import sys
import getopt
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge, CvBridgeError
from gazebo_msgs.msg import ModelState
from steplist2 import Foot,Steplist
from tf.transformations import quaternion_from_euler
import posix_ipc
import subprocess as prc
import mmap
import json
from walker import Walker, Astep
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from ipcomm import IPComm
from ihmc_msgs.msg import FootstepStatusRosMessage

MSG_INTERVAL = 0.05

class Flyer:
  def __init__(self,ip,port):

    self.bridge = CvBridge()
    self.imageflags = [0,0,0,0,0]  #I 1gray 2small 3fc 4seg (zero not used)
    self.imagecnt = 0
    self.key = 0
    self.height = 1.6
    self.tilt = 0.0
    self.torso = 0.0
    self.headtilt = 0.0
    self.headrot = 0.0
    self.stepspending = 0
    self.totalsteps = 0
    self.nextposetime = 0
    self.nextfsstime = 0
    memory = posix_ipc.SharedMemory("flyermmap", posix_ipc.O_CREAT, size=8*IPComm.Mib)
    self.sem = posix_ipc.Semaphore("flyersem", posix_ipc.O_CREAT)
    self.memmap = mmap.mmap(memory.fd, memory.size)
    memory.close_fd()
    self.image_sub = rospy.Subscriber("/olrun/image/gray",Image,self.callback)
    self.fcimage_sub = rospy.Subscriber("/olrun/image/fc",Image,self.callbackfc)
    self.segimage_sub = rospy.Subscriber("/olrun/image/seg",Image,self.callbackseg)
    self.smallimage_sub = rospy.Subscriber("/olrun/image/small",Image,self.callbacksmall)
    ROBOT_NAME = rospy.get_param('/ihmc_ros/robot_name')
    fss_name = "/ihmc_ros/{0}/output/footstep_status".format(ROBOT_NAME)
    pose_name = "/ihmc_ros/{0}/output/robot_pose".format(ROBOT_NAME)
    self.fssSubscriber = rospy.Subscriber( fss_name, FootstepStatusRosMessage, self.rcvdfss)
    self.poseSubscriber = rospy.Subscriber( pose_name, Odometry, self.rcvdpose )
    self.queue = posix_ipc.MessageQueue("/flyerqueue", posix_ipc.O_CREAT)
    self.wsproc = prc.Popen('python -u flyerws.py --ip %s --port %s' % (ip,port), shell=True )
    self.writecnt = 0
    self.loc = [0.0, 0.0, 0.0]
    self.walker = Walker()
    self.walker.init()
    self.neck_joint = [0.0, 0.0, 0.0]
    self.torso_joint = 0.0

  def rcvdpose(self,msg):
	now = rospy.Time.now().to_sec()
	if now > self.nextposetime:
	    self.nextposetime = now + MSG_INTERVAL
	    pos = msg.pose.pose.position
	    quat = msg.pose.pose.orientation
	    eul = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
	    self.loc = [float(pos.x),float(pos.y),float(eul[2])]

  def rcvdfss(self,msg):
	#now = rospy.Time.now().to_sec()
	#if now > self.nextfsstime:
	#    self.nextfsstime = now + SrcState.INTERVAL
	    if msg.status == 1:
		self.stepspending -= 1

  def callbackfc(self,msg):   #3fc
    num = 3
    if self.imageflags[num] == 0:
      try:
        self.fcimg = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        self.imageflags[num] = 1
      except CvBridgeError as e:
        print(e)

  def callbacksmall(self,msg):#2small
    num = 2
    if self.imageflags[num] == 0:
      try:
        self.smallimg = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        self.imageflags[num] = 1
      except CvBridgeError as e:
        print(e)

  def callbackseg(self,msg):  #4seg
    num = 4
    if self.imageflags[num] == 0:
      try:
        self.segimg = self.bridge.imgmsg_to_cv2(msg, "mono8")
        self.imageflags[num] = 1
      except CvBridgeError as e:
        print(e)

  def callback(self,msg):     #1-gray
    num = 1
    if self.imageflags[num] == 0:
      try:
        self.grayimg = self.bridge.imgmsg_to_cv2(msg, "mono8")
        self.imageflags[num] = 1
      except CvBridgeError as e:
        print(e)

  def get_image(self,num):
    self.imageflags[num] = 0  #I 1gray 2small 3fc 4seg
    timeout = time.time() + 1.0
    while self.imageflags[num] == 0 and time.time() < timeout:
      time.sleep(0.1)
    key = self.key
    self.key = 0
    return key

  def write_image(self,img,lock=True):
	if lock:
	    self.sem.acquire()
	self.memmap.seek(28)
	self.memmap.write(img)  #copy?
        l = self.memmap.tell()
	self.memmap.seek(0)
        self.memmap.write( np.array( [ l, self.writecnt ], 'i' ) )
	hd = [ self.loc[0], self.loc[1], self.loc[2], 
		float(self.totalsteps), float(self.stepspending) ]
        buf = np.array(hd,'f')
	self.memmap.write(buf)
	#print("Wrote %d bytes" % self.memmap.tell())
   	self.sem.release()
        self.writecnt += 1

  def check_queue(self):
    #X reset, I 1gray 2small 3fc 4seg, S step R,th, V step u/d, T torso, H head u/d, L hd look l/r
    found = False
    try:
      while self.queue.current_messages > 0:
	found = True
	#print("%s has %d msgs." % (self.queue.name,self.queue.current_messages))
	payload,prio = self.queue.receive(0)
        #print(payload)
	msg = json.loads(payload)
	#print(msg)
	cmd = str(msg.get('cmd',' '))
	r = float(msg.get('r',0.0))
	t = float(msg.get('t',0.0))
	#print(cmd,r,t)
	if cmd == 'X':  #X reset walker
	     self.reset_walker(r)
	if cmd == 'I':  #I 1gray 2small 3fc 4seg
	     self.send_image(r)
	if cmd == 'S':  #S step R,th
	    self.step_cam(r,t)
	if cmd == 'V':  #V step u/d
	    self.lift_cam(r)
	if cmd == 'T':  #T torso
	    self.tilt_body(r)
	if cmd == 'H':  #H head u/d
	    self.tilt_cam(r)
	if cmd == 'L':  #L hd look l/r
	    self.rotate_cam(r)
	if cmd == 'A':  #Abort walking
	    self.abort_walker(r)
    except Exception as e:
	print("Flyer Queue Exception %s"%e )
    return found

  def reset_walker(self,r):   #all walker cmds = rst-reset,wlk,sup-stepup,tlt-torso,nck-[3],abt
    self.send_walker('rst',r)

  def send_image(self,r):     #1-Gray 2-Small 3-FC(save) 4-Seg
    num = int(r)
    self.get_image(num)
    if num == 1:
	self.write_image(self.grayimg)
    elif num == 2:
	self.write_image(self.smallimg)
    elif num == 3:
     try:
        path = "../olr/fc%d.jpg" % self.writecnt
        print("saving image to %s"%path)
	cv2.imwrite(path,self.fcimg)
        self.writecnt += 1
     except Exception as e:
        print("saving image error %s"%e)
    elif num == 4:
	self.write_image(self.segimg)    

  def step_cam(self,r,th):
    self.send_walker('wlk',r,th)
    self.stepspending += 1
    self.totalsteps += 1

  def lift_cam(self,r):
    self.send_walker('sup',r)

  def tilt_body(self,r):
    self.torso_joint = r
    self.send_walker('tlt',self.torso_joint)

  def tilt_cam(self,r):
    if r > 0.0:           #down
      self.neck_joint[0] = r
      self.neck_joint[2] = 0.0
    if r <= 0.0:           #down
      self.neck_joint[2] = r
      self.neck_joint[0] = 0.0
    self.send_walker('nck',self.neck_joint)

  def rotate_cam(self,r):  #neck joint 1 l:r +0.8:-0.8
    self.neck_joint[1] = r
    self.send_walker('nck',self.neck_joint)

  def abort_walker(self,r):
    self.send_walker('abt',1)

  def send_walker(self,cmd,r,th=0.0):
    step = Astep(cmd,64)
    step.R = r
    step.t = th
    print(step)
    steps = [step]
    self.walker.process_keys(steps)

  def fini(self):
    try:
        self.wsproc.terminate()
	print("WebSockets terminated")
	self.memmap.close()
	self.queue.close()
	posix_ipc.unlink_shared_memory("flyermmap")
	self.sem.release()
	self.sem.unlink()
	self.queue.unlink()
    except:
	print("Error shutting down Flyer")

def main(args):
  rospy.init_node('Flyer3', anonymous=True)
  ip = "192.168.2.10"
  port = 9001
  try:
        opts, args = getopt.getopt(sys.argv[1:], "", ["ip=","port="])
  except getopt.GetoptError as err:
        print(str(err))  # will print something like "option -a not recognized"
        sys.exit(2)
  for o, a in opts:
        if o == "--ip":
            ip = a
        if o == "--port":
            port = a
  mod = Flyer(ip,port)
  r = rospy.Rate(10)
  key = 0
  try:
    mod.get_image(1) #1-Gray 2-Small 3-FC(save) 4-Seg
    mod.write_image("Start",lock=False)
    while not rospy.is_shutdown():
      found = mod.check_queue()
      r.sleep()
      #key = mod.get_image()
      #if found:
      #   mod.write_image()
      #if key == 27:
      #   rospy.signal_shutdown("Esc Pressed")

  except KeyboardInterrupt:
    print("Shutting down")
  mod.fini()
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
