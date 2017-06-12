from __future__ import nested_scopes
from POSH.behaviour import Behaviour 
from config import *

class Utility(Behaviour):
    
    def __init__(self, agent, profiler):
        Behaviour.__init__(self,agent,(),(),None,self)

    def s_fail(self):
        return False
    
    def s_succeed(self):
        return True
    
    def a_do_nothing(self):
        return True
    
    def s_one_step(self):
        if self.agent.Dying.is_alive:      
            self.agent.Grooming.a_decrease_groom_saturation_time()
            self.agent.Eating.a_decrease_food_saturation_time()
            self.agent.Drinking.a_decrease_drink_saturation_time()