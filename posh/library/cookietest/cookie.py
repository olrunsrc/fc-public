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

# behaviour base for POSH behaviours
from POSH import Behaviour

# Returns the behavior object
#def make_behavior(*args, **kw):
#    return [Behavior( *args, **kw)]

# Called when pyposh is shutting down
#def destroy_world():
#    pass
#
# Instances of this class lives inside an agent. Can be used to
# emulate agent memory
class CookieBehaviour(Behaviour):
    def __init__(self, agent):
        # initialise the behaviour by specifying that it provides the action
        # 'change_dir' and the senses 'see_cookie' and 'fail'. These have
        # to correspond to a method of the class.
        Behaviour.__init__(self, agent,
                           ('change_dir', ),
                           ('see_cookie', 'fail'))
        # These are behavior varibles
        # self.patience = 50
        self.base_dir = os.getcwd()
        self.cwd = self.base_dir
        self.cookie_name = 'cookie'

        # old -- Behaviours take care of this now
        # Parent links back to the Agent object
        # self.agent = agent # Handled by the Base Class
        #Base.__init__(self, *args, **kw)
        #self.act_dict = {}
        #self.sense_dict = {}
        #self.init_acts()
        #self.init_senses()
	
    # This method is called by the scheduled POSH implementation to make sure
    # that the behavior is ok every cycle. Returns False if everything is OK.
    # We can assign error codes or something similar.
    def check_error(self):
        return False

    # The agent has recieved a request for exit. Stop running everything.
    def exit_prepare(self):
        pass
        
#    def init_acts(self):
#        self.agent.add_act("change_dir", self.change_dir)

#    def init_senses(self):
#        self.agent.add_sense("see-cookie", self.see_cookie)
#        self.agent.add_sense("fail", lambda : 0)

    def see_cookie(self):
        curdir = self.cwd[len(self.base_dir):]
        try:
            os.stat('%s/%s' % (self.cwd, self.cookie_name))
        except:
            self.log.info("Cookie Not Found at %s"  % curdir)
            return False
        
        self.log.info("Found cookie at %s" % curdir)
        return True
    
    def fail(self):
        return False
        
    def change_dir(self):
        tmplist = os.listdir(self.cwd)
        dirlist = []

        for x in tmplist:
            if os.path.isdir(x):
                dirlist.append(x)
        if self.cwd != self.base_dir:
            dirlist.append("..")

        # We are at the basedir, but there are no directories to change to.
        if not len(dirlist):
            return False

        result = dirlist[random.randrange(len(dirlist))]
        if result == "..":
            self.cwd = os.path.dirname(self.cwd)
        else:
            self.cwd += "/"+result
            
        self.log.info("Looking for cookie in %s" % self.cwd)
        return True

    
