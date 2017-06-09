#!/usr/bin/env python
from __future__ import print_function



class step():

    LEFT = 0
    RIGHT = 1
    NULLSTEP = 

    def __init__(self,side=LEFT,off=NULLSTEP):
        self.side = side
        self.ss = 1.0
        self.ds = 0.5
        self.offset = off
        self.planned = NULLSTEP
        self.updated = NULLSTEP
        self.actual = NULLSTEP

class steplist():
    def __init__(self):
        self.fin = 0
        self.cur = 0
        self.pln = 0
        self.list = []
        self.msgno = 0

    def addstep(self,off):
	newstep = Step(off)
	self.list.append(newstep)
        self.pln = len(self.list)

    def sendplanned(self):
        pass

    def markcompleted(self,side,pose):
        pass


def testme():
    rospy.init_node('StepListTest')
    rate = rospy.Rate(10) # 10hz
    time.sleep(1)
    try:
	    thread.start_new_thread(web.run, ())
            while not rospy.is_shutdown():
		theSmartie.loop()
                rate.sleep()
    finally:
            rospy.loginfo("Smartie Died")

if __name__ == '__main__':
  testme()
