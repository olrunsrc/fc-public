from __future__ import nested_scopes

# for POSH.Behaviour
import POSH


# Global variables for the behavior

MAX_DIST_GROUP = 100   # maximum distance agent can be from closest agent 


class Exploring(POSH.Behaviour):
   
    def __init__(self, agent):    # initialises exploring behavior
       
        # FIXME: They're just here because they appear in the plan.
        # I don't know where they should actually go.
        self.fail = lambda: 0
        self.succeed = lambda: 1
       
        POSH.Behaviour.__init__(self, agent,            # registers its actions and senses
            ('move_around', 'move_to_closest'),
            ('wants_to_explore', 'is_far_from_group', 'succeed', 'fail'))

        
        # defines variables for behavior to starting configuration
        self.wander_target = None    # target we will wander to
        self.wander_steps = 1        # steps we will make to that target 
        
# ---------------------------------------------------------------------------
# Actions
# ---------------------------------------------------------------------------
    
    def move_around(self):        # Gets random location and moves random number of steps towards it.        
        self.wander_target = self.agent.MASON.getRandomLocation()
        self.wander_steps = self.random.randint(10, 30)
        self.agent.MASON.moveTo(self.wander_target, self.wander_steps)
            
     
    def move_to_closest(self):    # Looks for closest agent and moves until it is 3 units away from it.
        closest = self.agent.MASON.closest('agents')
        self.agent.MASON.moveTo(closest, 3)
        
        
# ---------------------------------------------------------------------------
# Senses
# ---------------------------------------------------------------------------
	
    def wants_to_explore(self):   # Returns 1 if the agent wants to explore. 
        return 1                  # Since exploring is default function, it is always one.      
      

    def is_far_from_group(self):  # Returns 1 if the agent reached max dist. to the next agent.          
        group_distance = \
            self.agent.MASON.distance(self.agent.MASON.closest('agents'))
        if group_distance >= MAX_DIST_GROUP:
            return 1
        else:         
            return 0
         
