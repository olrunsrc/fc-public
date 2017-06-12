from __future__ import nested_scopes

import POSH


FRUIT_MAX_LOAD = 100
FRUIT_MIN_LOAD = 10


class Growing(POSH.Behaviour):
    
    def __init__(self, agent):                            # initialises the growing behaviour

        POSH.Behaviour.__init__(self, agent,             # registers its actions and senses
            ('grow_food', 'stop_growing', ),                       
            ('full_tree', ))
                                                          
        # defines variables for behavior to starting configuration    
        self.fruit_load = self.random.randint (0, 100)
        
#        self.food_target = None                               # food target 
#        self.food_saturation = self.random.randint (50, 90)   # food_saturation, the smaller the number the more hungry 
#        self.after_eating_latch = 1                           # latch for whether I have eaten before
#        self.i_am_feeding = 0                                 # am I eating at the moment?
#        self.empty_food = []                                  # creates a list of food that was found to be maxed out;
#        self.exp = self.random.randint (0, 100)
        
        #sets inspectors
        self.registerInspectors(('FruitLoad', ))

# ---------------------------------------------------------------------------
# This defines the inspectors
# ---------------------------------------------------------------------------
    
    def getFruitLoad(self):
        return int(self.fruit_load)
    
    def setFruitLoad(self, level):
        self.fruit_load = float(level)


# ---------------------------------------------------------------------------
# Actions
# ---------------------------------------------------------------------------
    
 
    def grow_food(self):                          # increases saturation until it stops
        if self.fruit_load < FRUIT_MAX_LOAD:
            self.fruit_load += 1
        else:
            self.fruit_load += 0
    
    def stop_growing(self):
        self.fruit_load = 10
        return 0
           
#        if self.fruit_load <= FRUIT_MIN_LOAD:            
#            return 0
#        else:
#            return 1   
#    

# ---------------------------------------------------------------------------
# Senses
# ---------------------------------------------------------------------------
    
    def full_tree(self):    
        if self.fruit_load <= FRUIT_MIN_LOAD:
            return 0
        else:
            return 1


# misc functions, no actual behaviour
    def tree_is_full(self):
    #        if self.agent_capacity <= 0:
    #            return 1
        return 0