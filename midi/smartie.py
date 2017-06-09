#!/usr/bin/env python
#
# joystick input driver for Korg NanoKontrol input device
#
# Author: Austin Hendrix
# Author: Allison Thackston


import rospy
import rospkg
import rosparam

from borad import *
from axis import Axis
from walker import Walker
import pages
import thread
import os

class Smartie(object):

    ''' Class to connect to and publish the Korg NanoKontrol midi device
        as a ros joystick
    '''

    def __init__(self):
        ''' connect to midi device and set up ros
        '''
        self.sp = SmartPAD('SmartPAD')
        self.axis = Axis()
        self.axis.init()
        self.walker = Walker()
        self.walker.init()
	self.web = pages.Pages()

        rospy.loginfo("Using input device %s" % self.sp.device)
	os.environ['PORT']='8081'

    def finish(self):
        self.sp.finish()
        self.axis.fini()
	self.web.finish()
        #del self.sp
        #del self.axis

    def run(self):
        try:
	    thread.start_new_thread(self.web.run, ())
            while not rospy.is_shutdown():
                done,data,steps = self.sp.readin()
                if done:
                    rospy.signal_shutdown("Smartie Done")
                self.axis.process_keys(data)
                self.walker.process_keys(steps)
        finally:
            rospy.loginfo("Smartie Died")
            self.finish()

if __name__ == '__main__':
    rospy.init_node('Smartie2')
    teleop = Smartie()
    teleop.run()
