from __future__ import nested_scopes
from POSH.behaviour import Behaviour 
from POSH.latch import Latch
from config import *
from interrupt import interrupt

class Eating(Behaviour,Latch):
    
    def __init__(self, agent, profiler):      
        Behaviour.__init__(self,agent,(),(),None,self)
        profiler.register(self,eat_registered)
        Latch.__init__(self,self.random.randint(eat_rnd_lower,eat_rnd_upper),eat_lower_threshold,eat_increment,eat_decrement,eat_upper_threshold,eat_latch_threshold)     
        self.inter=interrupt(eat_upper_threshold,eat_increment-eat_decrement,eat_num_interrupts)
        
        self.food_target=None
        self.prev_food_target_loc=None
            
    def a_pick_food(self):
        oldNeighbourhoodSize=self.agent.MASON.getNeighbourhoodSize()
        self.food_target=None
        currentRadius=self.random.randint(80,120)
        maxRadius=600
        extendRadius=self.random.randint(80,120)
                    
        while self.food_target==None and currentRadius<=maxRadius:
            self.agent.MASON.setNeighbourhoodSize(currentRadius)
            
            neighbours=self.agent.MASON.neighbours('food',currentRadius)
            self.agent.random.shuffle(neighbours)
            
            for element in neighbours:
                if element.agent.Resources.s_has_food_left() and not element.loc==self.prev_food_target_loc:
                    self.food_target=element
                    break
            currentRadius+=extendRadius            
        self.agent.MASON.setNeighbourhoodSize(oldNeighbourhoodSize)
        
        if self.food_target==None:
            self.agent.Exploring.move_around()        

    def a_move_to_food(self): 
        self.agent.MASON.moveTo(self.food_target.loc,eat_move_distance)
        
        if not self.food_target.agent.Resources.s_has_food_left():
            self.food_target=None        
    
    def a_eat(self):
        if self.inter.should_interrupt(self.get_current_state()):
            self.prev_food_target_loc=self.food_target.loc
            self.food_target=None
            self.signal_interrupt()
            self.inter.increase_count()
            return 0  
        
        self.increment_current_state()
        self.food_target.agent.Resources.a_reduce_food_load()
        
        if not self.food_target.agent.Resources.s_has_food_left():
            self.signal_interrupt()
            self.food_target = None
        
        if self.is_saturated():          
            self.food_target = None
            self.prev_food_target_loc=None
            self.inter.reset_count()
            return 0
        
        return 1

    def a_decrease_food_saturation_time(self):          
        self.decrement_current_state()
        if self.failed():
            self.agent.Dying.died()
        return 1
            
    def s_wants_food(self):   
        return self.is_triggered() 

    def s_has_food(self):
        return (self.food_target != None)

    def s_is_near_food_target(self):                      
        if self.food_target and self.agent.MASON.distance(self.food_target)<2:
            self.activate()
            return 1
        return 0
    
    def signal_interrupt(self):
        if Latch.signal_interrupt(self):
            self.food_target=None