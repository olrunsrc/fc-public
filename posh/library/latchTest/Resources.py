from __future__ import nested_scopes
from POSH.behaviour import Behaviour 
from config import *

class Resources(Behaviour):
    
    def __init__(self,agent,profiler):      
        Behaviour.__init__(self,agent,(),(),None,self)
        
        self.current_food_load=res_max_load
        self.just_been_depleted=False
        self.just_been_restored=True
        self.counter=0
                               
    def depleted(self):
        pass
                                                              
    def a_reduce_food_load(self):
        self.current_food_load-=res_decrement
        
        if self.current_food_load<=0:
            self.current_food_load=0
            self.just_been_depleted=True
        
    def a_increase_food_load(self):        
        if self.just_been_depleted:
            self.counter+=1
            if self.counter%res_delay==0:
                self.just_been_depleted=False
                self.depleted()
                self.agent.MASON.setLoc(self.agent.MASON.getRandomLocation())
        else:
            self.current_food_load+=res_increment
        
        if self.current_food_load>res_max_load:
            self.current_food_load=res_max_load
            
    def a_delayed_restore_food_load(self):        
        if self.just_been_depleted:
            self.counter+=1
            if self.counter%res_delay==0:
                self.just_been_depleted=False
                self.just_been_restored=False
        elif not self.just_been_restored:   
            self.current_food_load=res_max_load
            self.just_been_restored=True
         
    def s_has_food_left(self):
        return self.current_food_load>0
    
    def s_is_occupied(self):
        neighbours=self.agent.MASON.neighbours('agents',2)
        return len(neighbours)>0