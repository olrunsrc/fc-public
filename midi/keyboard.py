#!/usr/bin/env python
#
#           Keyboard Teleop for Space Robotics Challenge 0.1.0
#           Copyright (C) 2017 Open Source Robotics Foundation
#           Released under the Apache 2 License
#
from __future__ import print_function

import select
import sys
import termios
import tty
import atexit

import rospy

class Keyboard:

    def __init__(self,logger=print):
	self.loginfo=logger  #pass in rospy.loginfo if you want
	pass

    def init(self):
        # save terminal settings
        self.settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
	self.newsettings = termios.tcgetattr(sys.stdin)
	self.newsettings[6][termios.VMIN] = 0
	self.newsettings[6][termios.VTIME] = 0

    #@atexit.register
    def fini(self):
        # restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def get_key(self):
        """Get input from the terminal."""
        #tty.setraw(sys.stdin.fileno())
        #select.select([sys.stdin], [], [], 0)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.newsettings)
        key = sys.stdin.read(1)
        return key

    def loginfo(self, msg):
        """Log info message while terminal is in funky mode."""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        self.loginfo(msg+"\r")
        #tty.setraw(sys.stdin.fileno())

def test():
    kb = Keyboard()
    print("Press ESC to exit.\n")
    kb.init()
    ch = ' '
    while not ch or not ord(ch)==27:
	ch = kb.get_key()
        if ch: kb.loginfo("key pressed %s\r" % str(ch))
    kb.fini()

def rostest():
    rospy.init_node('keyboard_test')
    kb = Keyboard(rospy.loginfo)
    kb.init()
    try:
	while not rospy.is_shutdown():
	    ch = kb.get_key()
            if ch: kb.loginfo("key pressed %s\r" % ch)
	    if ch and ord(ch)==27:
		rospy.signal_shutdown("Done")
    except rospy.ROSInterruptException:
	pass
    finally:
	kb.fini()
	rospy.loginfo("Exiting")

if __name__ == '__main__':
    #test()
    rostest()

