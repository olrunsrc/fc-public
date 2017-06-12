from __future__ import nested_scopes
from POSH.behaviour import Behaviour 
from POSH.latch import Latch
from config import *
from interrupt import interrupt

class Drinking(Behaviour,Latch):
    
    def __init__(self, agent, profiler):      
        Behaviour.__init__(self,agent,(),(),None,self)
        Latch.__init__(self,self.random.randint(dr_rnd_lower,dr_rnd_upper),dr_lower_threshold,dr_increment,dr_decrement,dr_upper_threshold,dr_latch_threshold)                                                                                                
        profiler.register(self,dr_registered)
        self.inter=interrupt(dr_upper_threshold,dr_increment-dr_decrement,dr_num_interrupts)
        
        self.drink_target=None 
        self.prev_drink_target_loc=None
        
    def a_pick_drink(self):         
        oldNeighbourhoodSize=self.agent.MASON.getNeighbourhoodSize()
        self.drink_target=None
        currentRadius=self.random.randint(80,120)
        maxRadius=600
        extendRadius=self.random.randint(80,120)
                    
        while self.drink_target==None and currentRadius<=maxRadius:
            self.agent.MASON.setNeighbourhoodSize(currentRadius)
            
            neighbours=self.agent.MASON.neighbours('water',currentRadius)
            self.agent.random.shuffle(neighbours)
            
            for element in neighbours:
                if element.agent.Resources.s_has_food_left() and not element.loc==self.prev_drink_target_loc:
                    self.drink_target=element
                    break
            currentRadius+=extendRadius            
        self.agent.MASON.setNeighbourhoodSize(oldNeighbourhoodSize)
        
        if self.drink_target==None:
            self.agent.Exploring.move_around()   
    
    def a_move_to_drink(self):
        self.agent.MASON.moveTo(self.drink_target.loc,dr_move_distance)
        
        if not self.drink_target.agent.Resources.s_has_food_left():
            self.drink_target=None
        
    def a_drink(self):
        if self.inter.should_interrupt(self.get_current_state()):
            self.prev_drink_target_loc=self.drink_target.loc
            self.drink_target=None
            self.signal_interrupt()
            self.inter.increase_count()
            return 0
        
        self.increment_current_state()
        self.drink_target.agent.Resources.a_reduce_food_load()
        
        if not self.drink_target.agent.Resources.s_has_food_left():
            self.signal_interrupt()
            self.drink_target=None
        
        if self.is_saturated():  
            self.drink_target=None
            self.prev_drink_target_loc=None
            self.inter.reset_count()
            return 0  
    
    def a_decrease_drink_saturation_time(self):             
        self.decrement_current_state()
        if self.failed():
            self.agent.Dying.died()
        return 1
        
    def s_wants_drink(self):
        return self.is_triggered() 

    def s_has_drink(self):
        return (self.drink_target!=None)

    def s_is_near_drink_target(self):
        if self.drink_target and self.agent.MASON.distance(self.drink_target)<2:
            self.activate()
            return 1
        return 0
    
    def signal_interrupt(self):
        if Latch.signal_interrupt(self):
            self.drink_target=None