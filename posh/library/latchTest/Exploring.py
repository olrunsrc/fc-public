from __future__ import nested_scopes

from POSH.behaviour import Behaviour 
from config import *

class Exploring(Behaviour):
   
    def __init__(self, agent, profiler):       
        Behaviour.__init__(self,agent,(),(),None,self)
        profiler.register(self,ex_registered)
        self.wander_target=None

    def a_move_around(self):        
        wander_target=self.agent.MASON.getRandomLocation()
        self.agent.MASON.moveTo(wander_target,ex_wander_steps)
        
    def move_around(self):        
        wander_target=self.agent.MASON.getRandomLocation()
        self.agent.MASON.moveTo(wander_target,ex_wander_steps)        