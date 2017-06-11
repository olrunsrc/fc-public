#!/usr/bin/env python
from __future__ import print_function
import roslib
import sys
import rospy
import cv2
import getopt
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self,name):
    self.image_pub = rospy.Publisher("/sample2",Image,queue_size=1)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber(name,Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    if rows < 134:
      cv_image = cv2.resize(cv_image, None, fx=(134.0/rows), fy=(134.0/rows))
      (rows,cols,channels) = cv_image.shape
    if cols > 60 and rows > 60 :
      cv2.circle(cv_image, (50,50), 10, 255)

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    try:
      pimg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
      #pimg.header.frame_id = 'world'
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
      print("Published image %d" % data.header.seq)
    except CvBridgeError as e:
      print(e)

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


def main(args):
  rospy.init_node('image_converter', anonymous=True)
  name, win = getoptions()
  ic = image_converter(name)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
