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
import math

# behaviour base for POSH behaviours
from POSH import Behaviour
from phil1 import TCPRobot

# Returns the behavior object
#def make_behavior(*args, **kw):
#    return [Behavior( *args, **kw)]

# Called when pyposh is shutting down
#def destroy_world():
#    pass
#
# Instances of this class lives inside an agent. Can be used to
# emulate agent memory
class PyroBehaviour(Behaviour):
    def __init__(self, agent):
        # initialise the behaviour by specifying that it provides the action
        # 'change_dir' and the senses 'see_cookie' and 'fail'. These have
        # to correspond to a method of the class.
        Behaviour.__init__(self, agent,
                           ('change_dir', ),
                           ('see_cookie', 'fail'))
        # These are behavior variables
        # self.patience = 50
        #self.base_dir = os.getcwd()
        #self.cwd = self.base_dir
        self.cookie_name = 'cookie'
        self.port = 0
        self.robot = None
        self.robotname = ""
        self.sonar = []
        self.pose = (0.0,0.0,0.0)
        self.others = []
        self.sensetm = time.time()
        self.sensedelay = 0.1

    def sensors(self):
        if not self.robot:
            self.log.info("Creating TCPRobot on port %s" %self.port)
            self.robot = TCPRobot("localhost",self.port)
            self.robotname = self.robot.send("name")
            self.log.info("%s : %s" % (self.robotname,self.robot.send("s_sonar_0")))
        stm = time.time()
        if stm > self.sensetm + self.sensedelay:
            self.sensetm = stm
            self.sonar = self.robot.send("sonar_0")
            self.pose = self.robot.send("c_%s" % self.robotname)
            res = self.robot.send("![(x.name,x._gx,x._gy,x._ga) for x in self.assoc.values()]")
            exec("poses = %s" % res)
            self.others = [ self.getObs(x) for x in poses if x[0] != self.robotname]
            #self.log.info(self.sonar)
            #self.log.info(self.others)

    def getObs(self,p):
            name = p[0]
            dx = p[1] - self.pose[0]
            dy = p[2] - self.pose[1]
            r = math.sqrt(dx*dx + dy*dy)
            th = math.pi/2.0 if dy == 0 else math.atan(dx/dy)
            return (name,r,th)
	
    # This method is called by the scheduled POSH implementation to make sure
    # that the behavior is ok every cycle. Returns False if everything is OK.
    # We can assign error codes or something similar.
    def check_error(self):
        return False

    # The agent has recieved a request for exit. Stop running everything.
    def exit_prepare(self):
        self.log.info(self.robot.send("exit"))

    def see_cookie(self):
        self.sensors()
        range = 1.0
        #scwd = self.cwd[len(self.base_dir):]
        #try:
        #    os.stat('%s/%s' % (self.cwd, self.cookie_name))
        #except:
        #    self.log.info("Cookie Not Found at %s"  % scwd)
        #    return False
        
        #self.log.info("Found cookie at %s" % scwd)
        #self.log.info(self.sonar)
        if len(self.sonar) > 4:
            range = min(1.0,self.sonar[3],self.sonar[4])
        if range < 0.5:
            self.robot.send("m_0_0")
            self.log.info("Found cookie at %s" % range)
        #time.sleep(1)
        return range < 0.5
    
    def fail(self):
        return False
        
    def change_dir(self):
        self.sensors()
        #time.sleep(1)
        #tmplist = os.listdir(self.cwd)
        #dirlist = []

        #for x in tmplist:
        #    if os.path.isdir(x):
        #        dirlist.append(x)
        #if self.cwd != self.base_dir:
        #    dirlist.append("..")

        # We are at the basedir, but there are no directories to change to.
        #if not len(dirlist):
        #    return False

        #result = dirlist[random.randrange(len(dirlist))]
        #if result == "..":
        #    self.cwd = os.path.dirname(self.cwd)
        #else:
        #    self.cwd += "/"+result

        #scwd = self.cwd[len(self.base_dir):]            
        #self.log.info("Looking for cookie in %s" % scwd)
        self.robot.send("m_1_0")
        return True

    
