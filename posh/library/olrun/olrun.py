#!/usr/bin/python
#
#  Testing behavior
#
#  Changes directory randomly and provide respose for seeing cookie
#

# For compatibility with Jython
from __future__ import nested_scopes
from POSH.jython_compat import *

import os
import random
import time
import sys

# behaviour base for POSH behaviours
from POSH import Behaviour

# tasknum, taskname, timeout, cycles, retries
TaskList =         [ (1.0, 'startup'     , 60, 15, 99 ),
                     (1.1, 'go2sat'      ,   1,  8, 2 ),
                     (1.2, 'adjustyaw'   ,   1,  8, 2 ), 
                     (1.3, 'adjustpitch' ,   1,  8, 2 ),
                     (1.4, 'go2fin1'     ,   1,  8, 2 ),
                     (2.0, 'go2panel'    ,   1,  8, 2 ),
                     (2.1, 'liftpanel'   ,   1,  8, 2 ),
                     (2.2, 'placepanel'  ,   1,  8, 2 ),
                     (2.3, 'pressbutton' ,   1,  8, 2 ),
                     (2.4, 'liftcable'   ,   1,  8, 2 ),
                     (2.5, 'plugincable' ,   1,  8, 2 ),
                     (2.6, 'go2fin2'     ,   1,  8, 2 ),
                     (3.0, 'go2stairs'   ,   1,  8, 2 ),
                     (3.1, 'climbstairs' ,   1,  8, 2 ),
                     (3.2, 'opendoor'    ,   1,  8, 2 ),
                     (3.3, 'go2table'    ,   1,  8, 2 ),
                     (3.4, 'liftdetector',   1,  8, 2 ),
                     (3.5, 'findleak'    ,   1,  8, 2 ),
                     (3.6, 'liftrepair'  ,   1,  8, 2 ),
                     (3.7, 'repairleak'  ,   1,  8, 2 ),
                     (3.8, 'go2fin3'     ,   1,  8, 2 ),
                     (3.9, 'alldone'     , 60, 10, 99 )]

def thisTask(task):
    indexes = [ i[0] for i in TaskList ]
    this = TaskList[indexes.index(task)]
    return this

def nextTask(task):
    indexes = [ i[0] for i in TaskList ]
    next = TaskList[indexes.index(task)+1]
    return next

class OlrunBehaviour(Behaviour):
    def __init__(self, agent):
        Behaviour.__init__(self, agent,
        # actions:
                   ( 'startup', 'go2sat', 'adjustyaw', 'adjustpitch', 'go2fin1', 'go2panel', 'liftpanel',
                     'placepanel', 'pressbutton', 'liftcable', 'plugincable', 'go2fin2', 'go2stairs',
                     'climbstairs', 'opendoor', 'go2table', 'liftdetector', 'findleak', 'liftrepair',
                     'repairleak', 'go2fin3', 'alldone', 'retry','prep4Task1_1','phone_home', 'reharness',
                     'prep4Next' ),
        # senses:
                   ('see_sat','fallen','timedout','retries_left','currentTaskIs','fail','succeed'))
        # These are behavior variables
        # self.agent = agent # Handled by the Base Class
        self.Timeout = 0
        self.retryCnt = 0
        self.startupTime = time.time()  #This should be wall time.  All others ROS time.
        self.curTask = -1
        self.stepCnt = 0
	
    # This method is called by the scheduled POSH implementation to make sure
    # that the behavior is ok every cycle. Returns False if everything is OK.
    # We can assign error codes or something similar.
    def check_error(self):
        return False

    # The agent has received a request for exit. Stop running everything.
    def exit_prepare(self):
        pass

    def prep4Task(self,tasknum,timeout=3,cycles=10,retries=1):
        print("Prepare for Task %3.1f." % (tasknum))
        self.curTask = tasknum
        self.Timeout = timeout
        self.timeoutTime = time.time() + self.Timeout
        self.stepCnt = cycles
        self.retryCnt = retries
        return True

    def prep4Next(self):
        next = nextTask(self.curTask)
        print("Next checkpoint is %3.1f - %s" % (next[0],next[1]))
        self.prep4Task(next[0],next[2],next[3],next[4])
        extrawork = getattr( self.__class__, 'prep4'+next[1], None )
	if extrawork:
            extrawork(self)
        return True

    def reharness(self):
        print("Wait for reharness and detach")
        time.sleep(1)

    def prep4Task1_1(self): return self.prep4Task(1.1)
    def prep4go2sat(self): print("Prep for go2sat worked!")

    def startup(self): self.wasteTime('Startup')
    def go2sat(self): self.wasteTime('Go2Sat')
    def adjustyaw(self): self.wasteTime('AdjustYaw')
    def adjustpitch(self): self.wasteTime('AdjustPitch')
    def go2fin1(self): self.wasteTime('Go2Fin1')
    def go2panel(self): self.wasteTime('Go2Panel')
    def liftpanel(self): self.wasteTime('LiftPanel')
    def go2array(self): self.wasteTime('Go2Array')
    def placepanel(self): self.wasteTime('PlacePanel')
    def pressbutton(self): self.wasteTime('PressButton')
    def liftcable(self): self.wasteTime('LiftCable')
    def plugincable(self): self.wasteTime('PlugInCable')
    def go2fin2(self): self.wasteTime('Go2Fin2')
    def go2stairs(self): self.wasteTime('Go2Stairs')
    def climbstairs(self): self.wasteTime('ClimbStairs')
    def opendoor(self): self.wasteTime('OpenDoor')
    def go2table(self): self.wasteTime('Go2Table')
    def liftdetector(self): self.wasteTime('LiftDetector')
    def findleak(self): self.wasteTime('FindLeak')
    def liftrepair(self): self.wasteTime('LiftRepair')
    def repairleak(self): self.wasteTime('RepairLeak')
    def go2fin3(self): self.wasteTime('Go2Fin3')
    def alldone(self): 
        print('Done. Elapsed %6.2f seconds' % (time.time()-self.startupTime))
        self.curTask = 4.0

    def wasteTime(self, name):
        print("Working %s. Cycle %d. %6.2f secs remain." % (name,self.stepCnt,self.timeoutTime-time.time()) )
        time.sleep(1.0)
        self.stepCnt -= 1
        if self.stepCnt < 1:
            return True
        return False

    def phone_home(self):
        print("Phoning home.")
	time.sleep(2)
        return True

    def currentTaskIs(self):
        return self.curTask
    
    def retry(self):
        cur = thisTask(self.curTask)
        self.retryCnt -= 1
        print("Retry task %s, %d left" % (cur[1],self.retryCnt))
        self.timeoutTime = time.time() + self.Timeout
        self.stepCnt = cur[3]
        time.sleep(0.5)
        return True
    
    def see_sat(self):
         if self.stepCnt > 0:
             print("Don't see Sat.")
             return False
         return True
    
    def fallen(self):
        #print("Check fallen")
        return False
    
    def timedout(self):
        #print("Check timeout")
	if time.time() > self.timeoutTime:
            print("Checkpoint %2.1f timed out." % self.curTask)
            return True
        return False
    
    def retries_left(self):
        return self.retryCnt

    def succeed(self):
        return True

    def fail(self):
        return False



