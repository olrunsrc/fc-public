#!/usr/bin/env python
from __future__ import print_function

import rospy
import cv2
import time
import sys
import getopt
import numpy as np
from std_msgs.msg import UInt8MultiArray, MultiArrayLayout, MultiArrayDimension
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge, CvBridgeError
from tensornet import TensorNet

class Vision:
  def __init__(self,net,win):
    self.bridge = CvBridge()
    self.imageflag = 0  #poor lock for single image
    self.imagecnt = 0   #images received
    self.key = 0        #not used
    self.writecnt = 0   #images sent
    self.net = net      #NN image processor
    self.showwin = win  #show image window(s)
    self.r5_8uc3 = None #raw image from Val -- upside down BGR bytes 544x1024
    self.header  = None
    self.image_sub = rospy.Subscriber("/multisense/camera/left/image_raw",Image,self.callback)
    self.fcimage_pub = rospy.Publisher("/olrun/image/fc",Image,queue_size=1)
    self.grayimage_pub = rospy.Publisher("/olrun/image/gray",Image,queue_size=1)
    self.smallimage_pub = rospy.Publisher("/olrun/image/small",Image,queue_size=1)
    self.segimage_pub = rospy.Publisher("/olrun/image/seg",Image,queue_size=1)
    self.ratimage_pub = rospy.Publisher("/olrun/image/rat",UInt8MultiArray,queue_size=1)

  def callback(self,msg):
    self.imagecnt += 1
    if self.imageflag == 0:
      try:
        self.header = msg.header
        self.r5_8uc3 = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.imageflag = 1
      except CvBridgeError as e:
        print(e)

  def get_image(self):
    self.imageflag = 0
    timeout = time.time() + 1.0
    while self.imageflag == 0 and time.time() < timeout:
      time.sleep(0.1)

  def process(self):
    key = 0
    self.get_image()
    im8uc3 = cv2.flip(self.r5_8uc3[::2,::2],-1)  #272x512 rightside up, colors still scrambled
    fcimg = im8uc3 #cv2.cvtColor(im8uc3,cv2.COLOR_BGR2RGB)     #272x512 RGB bytes (408kB)
    fcmsg = self.bridge.cv2_to_imgmsg(fcimg, "rgb8")
    fcmsg.header = self.header
    grayimg = cv2.cvtColor(im8uc3[::2,::2],cv2.COLOR_RGB2GRAY)     #136x256 grayscale 1 byte (34kB)
    graymsg = self.bridge.cv2_to_imgmsg(grayimg, "mono8")
    graymsg.header = self.header
    smallimg = fcimg[::8,::8]     #34x64 RGB bytes (6516 Bytes)
    smallmsg = self.bridge.cv2_to_imgmsg(smallimg, "rgb8")
    smallmsg.header = self.header
    im32fc3 = fcimg.astype('float32')            #272x512 RBB float for NN input
    segimg = self.net.process(im32fc3)           #34x64   8bit output from NN  (2172 Nibbles/Bytes)
    segmsg = self.bridge.cv2_to_imgmsg(segimg, "mono8")
    segmsg.header = self.header
    ratimg = self.net.subset(segimg)             #96? bytes for navigation
    ratmsg = UInt8MultiArray()
    dim0 = MultiArrayDimension('Seg96',len(ratimg),len(ratimg))    #??? dim[0].name, size, stride
    ratmsg.layout.dim.append(dim0)
    ratmsg.layout.data_offset = 1 
    ratmsg.data = [self.writecnt] + list(ratimg)
    self.fcimage_pub.publish(fcmsg)
    self.grayimage_pub.publish(graymsg)
    self.smallimage_pub.publish(smallmsg)
    self.segimage_pub.publish(segmsg)
    self.ratimage_pub.publish(ratmsg)
    #print(ratmsg)
    self.writecnt += 1
    if self.showwin:
      cv2.imshow("Full Color", fcimg)
      key = cv2.waitKey(3) & 0xFF
    return key

  def fini(self):
    try:
        pass
    except:
	print("Error shutting down Vision")

def getoptions():
  netname = "Null"
  windows = False
  try:
        opts, args = getopt.getopt(sys.argv[1:], "n:w", ["netname="])
  except getopt.GetoptError as err:
        print(str(err))  # will print something like "option -a not recognized"
        sys.exit(2)
  for o, a in opts:
        if o == "--netname":
            netname = a
        if o == "-n":
            netname = a
        if o == "-w":
            windows = True
  return netname, windows

def main():
  rospy.init_node('Vision', anonymous=True)
  netname,win = getoptions()
  net = TensorNet(netname)
  mod = Vision(net,win)
  r = rospy.Rate(5)
  try:
    while not rospy.is_shutdown():
      mod.process()
      r.sleep()

  except KeyboardInterrupt:
    print("Shutting down")
  mod.fini()
  net.fini()
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
