#!/usr/bin/env python
from __future__ import print_function
import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import time
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Empty
from srcsim.msg import Console
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def keypt(k):
    return {'pt':k.pt,'size':k.size,'angle':k.angle}
def mvpt(p,r):
  return (int(p[0]+r), int(p[1]+r))
def pteps(p1,p2):
  return max(abs(p1[0]-p2[0]),abs(p1[1]-p2[1]))
def getkeypt(keys):
  if keys != []:
      pt1 = mvpt(keys[0].pt,-1*keys[0].size)
      pt2 = mvpt(keys[0].pt,   keys[0].size)
  else:
      pt1 = ( 10, 10)
      pt2 = mvpt(pt1, 10)
  return pt1,pt2

class image_converter:

  def __init__(self):
    self.start_pub = rospy.Publisher("/srcsim/qual1/start",Empty,queue_size=1)
    self.light_pub = rospy.Publisher("/srcsim/qual1/light",Console,queue_size=1)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/multisense/camera/left/image_raw",Image,self.callback)
    self.imageflag = 0
    self.imagecnt = 0
    self.background = self.get_hls()
    self.blobdet = cv2.SimpleBlobDetector(self.set_blob_params())
    self.screenkeys = []
    self.lightkeys = []
    self.lightptx = 0
    self.lightpty = 0
    self.lighttim = 0
    self.lightpt = (0,0)

  def set_blob_params(self):
    param = cv2.SimpleBlobDetector_Params()
    #param.minThreshold = 128
    #param.maxThreshold = 255
    param.filterByArea = True
    param.maxArea = 50000
    param.blobColor = 255
    return param

  def callback(self,data):
    roi = (90,300,384,384) #valkyrie image is 544x1024
    self.imagecnt += 1
    if self.imageflag == 0:
      try:
        im8uc3 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        #(rows,cols,channels) = im8uc3.shape
        if self.imagecnt == 10:
           print(im8uc3.shape )
           print(self.cv_image.shape)
           print(self.cv_image.dtype)
        im32fc3 = im8uc3.astype('float32')
        self.cv_image = (im32fc3/255.0)[roi[0]:roi[0]+roi[2], roi[1]:roi[1]+roi[3]]
        self.imageflag = 1
      except CvBridgeError as e:
        print(e)
    
    #(rows,cols,channels) = self.cv_image.shape
    #if cols > 60 and rows > 60 :
    #  cv2.circle(self.cv_image, (50,50), 10, 255)

    cv2.imshow("Image window", self.cv_image)
    cv2.waitKey(3)

  def get_image(self):
    self.imageflag = 0
    timeout = time.time() + 1.0
    while self.imageflag == 0 and time.time() < timeout:
      time.sleep(0.1)
    return self.cv_image

  def get_hls(self):
    bgr32 = cv2.resize(self.get_image(),(384,384))
    #hls = np.zeros((384,384,3), np.float)
    hls = cv2.cvtColor( bgr32, cv2.COLOR_BGR2HLS )
    self.hue, self.light, self.sat = cv2.split( hls )
    #return cv2.GaussianBlur( self.sat, (3,3), 0 )
    return cv2.medianBlur( self.sat, 5)

  def find_screen(self):
    wht = cv2.inRange(self.cv_image,(0.7,0.7,0.7),(1.0,1.0,1.0)) #result is CV_8UC1
    keys = self.blobdet.detect(wht)
    #if self.imagecnt == 10:
    #   cv2.imwrite('wht.png',wht)
    #if keys != self.keys:
    #  self.keys = keys
    #  print([keypt(i) for i in keys])
    #if keys != []:
    #  pt1 = mvpt(keys[0].pt,-1*keys[0].size)
    #  pt2 = mvpt(keys[0].pt,   keys[0].size)
    #else:
    #  pt1 = ( 10, 10)
    #  pt2 = mvpt(pt1, 10)
    pt1,pt2 = getkeypt(keys)
    self.screenkeys = keys
    #cv2.rectangle(wht,pt1,pt2,127,thickness=3)
    #cv2.imshow("White window", wht)
    #cv2.waitKey(3)
    return pt1,pt2
    
  def findlight(self,skeys,lkeys):
     ltpt = (0,0)
     if skeys != [] and lkeys != []:
        cx = skeys[0].pt[0]
        cy = skeys[0].pt[1]
        radius = skeys[0].size
        x = lkeys[0].pt[0]
        y = lkeys[0].pt[1]
        lx = (66.5/radius)*(x-cx)
        ly = (66.5/radius)*(y-cy)
        self.lightptx += int(lx)
        self.lightpty += int(ly)
        self.lighttim += 1
        ltpt = (int(self.lightptx/self.lighttim),int(self.lightpty/self.lighttim))
     else:
        self.lightptx = 0
        self.lightpty = 0
        self.lighttim = 0
        ltpt = (0,0)
     #if ltpt != self.lightpt:
     if pteps(ltpt,self.lightpt)>1:
        self.lightpt = ltpt
        print(self.imagecnt,ltpt)

  def compare_background(self, reset=False, adapt=0.0 ):
    val = self.get_hls()
    if reset:
       adapt = 1.0
    self.background = cv2.addWeighted(self.background,1.0-adapt,val,adapt,0)

    compare = val - self.background
    test = np.array( 255*compare, dtype=np.uint8 )
    keys = self.blobdet.detect(test)
    
    pt1,pt2 = getkeypt(keys)
    self.lightkeys = keys

    self.findlight(self.screenkeys,self.lightkeys)

    cv2.rectangle(compare,pt1,pt2,255,1)
    cv2.putText( compare, str(self.imagecnt), (10,30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2, cv2.CV_AA )
    pt1,pt2 = self.find_screen()
    cv2.rectangle(compare,pt1,pt2,127,1)
    cv2.imshow("Compare window", compare)
    cv2.waitKey(3)

  def send_start(self):
    self.start_pub.publish()

#    try:
#      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
#    except CvBridgeError as e:
#      print(e)

def main(args):
  rospy.init_node('qualtask1', anonymous=True)
  ic = image_converter()
  r = rospy.Rate(10)
  try:
    #rospy.spin()
    while not rospy.is_shutdown():
      ic.compare_background(reset=(ic.imagecnt==1), adapt=0.0 )
      if ic.imagecnt == 100:
        ic.send_start()
        pass
      r.sleep()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
