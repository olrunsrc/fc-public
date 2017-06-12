from __future__ import nested_scopes
from POSH.behaviour import Behaviour
from POSH.latch import Latch
from config import *

class Grooming(Behaviour,Latch):

    def __init__(self, agent, profiler):        
        Behaviour.__init__(self,agent,(),(),None,self)
        Latch.__init__(self,self.random.randint(gr_rnd_lower,gr_rnd_upper),gr_lower_threshold,gr_increment,gr_decrement,gr_upper_threshold,gr_latch_threshold)           
        profiler.register(self,gr_registered)       
        
        self.groom_target=None

    def a_pick_groom_target(self):            
        oldNeighbourhoodSize = self.agent.MASON.getNeighbourhoodSize()
        self.groom_target=None
        currentRadius=self.random.randint(80,120)
        maxRadius=600
        extendRadius=self.random.randint(80,120)
                    
        while self.groom_target == None and currentRadius <= maxRadius:
            self.agent.MASON.setNeighbourhoodSize(currentRadius)
            
            neighbours=self.agent.MASON.neighbours('agents',currentRadius)
            self.agent.random.shuffle(neighbours)
            
            for element in neighbours:
                if element.agent!=self.agent and element.agent.Grooming.s_wants_to_groom():
                    self.groom_target=element
                    break
            currentRadius+=extendRadius
        self.agent.MASON.setNeighbourhoodSize(oldNeighbourhoodSize)
        
        if self.groom_target==None:
            self.agent.Exploring.move_around()
                        
    def a_move_to_groom_target(self):            
        self.agent.MASON.moveTo(self.groom_target,gr_move_distance)
        
        if not self.groom_target.agent.Grooming.s_wants_to_groom():
            self.groom_target=None
            
    def a_groom_with_target(self):             
        self.increment_current_state()
        
        if not self.groom_target.agent.Grooming.s_wants_to_groom():
            self.signal_interrupt()
            self.groom_target=None
            return 0
        
        if self.is_saturated():          
            self.groom_target=None
            return 0
        
        return 1                                    
        
    def a_decrease_groom_saturation_time(self):
        self.decrement_current_state()
        return 1

    def s_wants_to_groom(self):
        return self.is_triggered() 
        
    def s_has_groom_target(self):
        return self.groom_target!=None    
    
    def s_is_near_groom_target(self): 
        if self.groom_target and self.agent.MASON.distance(self.groom_target)<2:
            self.activate()
            return 1
        return 0    
    
    def signal_interrupt(self):
        if Latch.signal_interrupt(self):
            self.groom_target=None    