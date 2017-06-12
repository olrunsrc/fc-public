from __future__ import nested_scopes
from POSH.behaviour import Behaviour 
from config import *

class Dying(Behaviour):
    
    def __init__(self, agent, profiler):
        Behaviour.__init__(self,agent,(),(),None,self)
        profiler.register(self,die_registered)
        
        self.is_alive=True
    
    def a_stay_dead(self):
        return True
        
    def died(self):
        self.is_alive=False
        self.agent.Grooming.reset_agent()
        self.agent.Drinking.reset_agent()
        self.agent.Eating.reset_agent()        
        
    def s_is_dead(self):
        return self.is_alive
