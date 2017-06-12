from __future__ import nested_scopes

import POSH


FRUIT_MAX_LOAD = 100
FRUIT_MIN_LOAD = 0



class Eaten(POSH.Behaviour):
    
    def __init__(self, agent):                            # initialises the growing behaviour
        POSH.Behaviour.__init__(self, agent,             # registers its actions and senses
            ('reduce_fruit_load',  ),                       
            ('vacant', ))        # defines variables for behavior to starting configuration    
        self.fruit_load = self.random.randint (0, 100)
        


# ---------------------------------------------------------------------------
# Actions
# ---------------------------------------------------------------------------
    
 
    def reduce_fruit_load(self):                 # increases saturation until it stops
        self.fruit_load -= 1
        return 1           
        if self.fruit_load <= FRUIT_MIN_LOAD:            
            return 0
        else:
            return 1   
    

# ---------------------------------------------------------------------------
# Senses
# ---------------------------------------------------------------------------
    
    def vacant(self):
        for element in self.agent.MASON.location('agents', ):    
            if element.agent != self.agent:                     
                return 0
            else:
                return 1

