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


#global flyermmap
Kib = 1024
Mib = Kib*Kib

#class Onestep(Astep):
#    def __init__(self,r,th):
#        super(Onestep,self).__init__('wlk',64)
#        self.R = r
#        self.t = th
#	self.lupd = 0

class Flyer:
  def __init__(self,ip,port):
#    global flyermmap
    print(posix_ipc.MESSAGE_QUEUES_SUPPORTED)
    print(posix_ipc.QUEUE_MESSAGES_MAX_DEFAULT)
    print(posix_ipc.QUEUE_MESSAGE_SIZE_MAX_DEFAULT)
    self.model_pub = rospy.Publisher("/gazebo/set_model_state",ModelState,queue_size=1)
    self.bridge = CvBridge()
    #self.image_sub = rospy.Subscriber("/mycam/image_raw",Image,self.callback)
    self.image_sub = rospy.Subscriber("/multisense/camera/left/image_raw",Image,self.callback)
    self.imageflag = 0
    self.imagecnt = 0
    self.key = 0
    firstFoot = Foot([0,-0.1,0],Foot.RIGHT)
    standingFoot = Foot([0,0.1,0],Foot.LEFT)
    self.list = Steplist(firstFoot,standingFoot)
    self.height = 1.6
    self.tilt = 0.0
    memory = posix_ipc.SharedMemory("flyermmap", posix_ipc.O_CREAT, size=8*Mib)
    self.sem = posix_ipc.Semaphore("flyersem", posix_ipc.O_CREAT)
    self.memmap = mmap.mmap(memory.fd, memory.size)
#    flyermmap = self.memmap
    memory.close_fd()
    self.queue = posix_ipc.MessageQueue("/flyerqueue", posix_ipc.O_CREAT)
    #self.wsproc = prc.Popen('python flyerws.py', shell=True )
    self.wsproc = prc.Popen('python flyerws.py --ip %s --port %s' % (ip,port), shell=True )
    self.writecnt = 0
    self.loc = (0,0,0)
    self.walker = Walker()
    self.walker.init()

  def callback(self,msg):
    #roi = (90,300,384,384) #valkyrie image is 544x1024
    roi = (0,0,544,1024) #valkyrie image is 544x1024
    self.imagecnt += 1
    if self.imageflag == 0:
      try:
        im8uc3 = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        im32fc3 = im8uc3.astype('float32')
        #self.cv_image = (im32fc3/255.0)[roi[0]:roi[0]+roi[2], roi[1]:roi[1]+roi[3]]
        #self.cv_image = im8uc3
        #self.cv_image = im8uc3[::2,::2,:] #272x512
        self.cv_image = im8uc3[::16,::16,:] #34x64
        self.imageflag = 1
      except CvBridgeError as e:
        print(e)
    '''
    cv2.imshow("Image window", self.cv_image)
    key = cv2.waitKey(3) & 0xFF
    if key != 255:
	self.key = key
	if key != 27: print(" key %d %s "%(self.key,chr(self.key)))
   '''

  def get_image(self):
    self.imageflag = 0
    timeout = time.time() + 1.0
    while self.imageflag == 0 and time.time() < timeout:
      time.sleep(0.1)
    key = self.key
    self.key = 0
    return key

  def write_image(self,lock=True):
    scale = 1.0/2.0
    if (True):
	if lock:
	    self.sem.acquire()
	self.memmap.seek(28)
	#thumb = cv2.resize(self.cv_image,None,0,scale,scale,cv2.INTER_AREA)
        #thumb = self.cv_image[::2,::2,:]
	#self.memmap.write(thumb.copy())
	self.memmap.write(self.cv_image.copy())
        l = self.memmap.tell()
	self.memmap.seek(0)
        self.memmap.write( np.array( [ l, self.writecnt ], 'i' ) )
	hd = [ self.loc[0], self.loc[1], self.loc[2], 
		self.height, self.tilt ]
        buf = np.array(hd,'f')
	self.memmap.write(buf)
	#print("Wrote %d bytes" % self.memmap.tell())
   	self.sem.release()
    self.writecnt += 1

  def check_queue(self):
    found = False
    try:
      while self.queue.current_messages > 0:
	found = True
	print("%s has %d msgs." % (self.queue.name,self.queue.current_messages))
	payload,prio = self.queue.receive(0)
        #print(payload)
	msg = json.loads(payload)
	#print(msg)
	cmd = str(msg.get('cmd','X'))
	r = float(msg.get('r',0.0))
	t = float(msg.get('t',0.0))
	if cmd == 'A':  #Absolute x,y,z, rotation, tilt
	    x = float(msg.get('x',0.0))
	    y = float(msg.get('y',0.0))
	    z = float(msg.get('z',0.0))
	    self.loc = (x, y, r)
	    self.height = z
	    self.tilt   = t
	    self.update_cam()
	if cmd == 'S':
	    self.step_cam(r,t)
	if cmd == 'V':
	    self.lift_cam(r)
	if cmd == 'T':
	     self.tilt_cam(r)
    except Exception as e:
	print("Flyer Queue Exception %s"%e )
    return found

  def step_cam(self,r,th):
    self.loc = self.list.fstep(r,th)
    #self.update_cam()#{key: 'wlk', R: 50, t: }
    step = Astep('wlk',64)
    step.R = r
    step.t = th
    print(step)
    steps = [step]
    self.walker.process_keys(steps)

  def tilt_cam(self,th):
    tl = self.tilt + th
    self.tilt = min(0.52,max(-0.52,tl))
    #self.update_cam()

  def lift_cam(self,r):
    ht = self.height + r
    self.height = min(5.0,max(1.2,ht))
    #self.update_cam()

  '''
  def update_cam(self):
    loc = self.loc
    print("stepped to (%6.3f,%6.3f) heading %6.3f" % loc)
    pose = Pose()
    pose.position.x = loc[0]
    pose.position.y = loc[1]
    pose.position.z = self.height
    q = quaternion_from_euler( 0, self.tilt, loc[2] )
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]
    msg = ModelState(model_name='mycam',pose=pose)
    self.model_pub.publish(msg)
  '''

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
  rospy.init_node('Flyer2', anonymous=True)
  ip = "10.9.20.1"
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
    mod.get_image()
    mod.write_image(lock=False)
    while not rospy.is_shutdown():
      found = mod.check_queue()
      r.sleep()
      key = mod.get_image()
      if found:
         mod.write_image()
      if key == 27:
	 rospy.signal_shutdown("Esc Pressed")

  except KeyboardInterrupt:
    print("Shutting down")
  mod.fini()
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
