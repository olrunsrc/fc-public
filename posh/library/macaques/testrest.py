from __future__ import nested_scopes

# for POSH.Behaviour
import POSH

# REST_MAX_VALUE = 100
# REST_MIN_VALUE = 20

# avoid behaviour from being loaded
#class Resting(POSH.Behaviour):
    """ Resting Behaviour.
    """
    def __init__(self, agent):
        """Initialises the Resting behaviour.
        """
        # registers all its actions and senses
        POSH.Behaviour.__init__(self, agent,
            ('resting', ),
            ('want_to_rest', ))
        # set initial state
        self.resting_time = 0    # resting time is time agent will not move (randomly choosen between 10 and 30 and then reduced every time step) 
        self.resting_status = 0  # whether the agent is resting or not
        # Set the inspectors
        self.registerInspectors(('RestingTime', ))

# ---------------------------------------------------------------------------
# Accessors and Mutators
# ---------------------------------------------------------------------------
    
    def getRestingTime(self):
        return self.resting_time
    
# ---------------------------------------------------------------------------
# Actions
# ---------------------------------------------------------------------------
    
    def resting(self):
        """Lets the agent rest.
        
        I.e. that doesn't do anything.
        """
        return 1

# ---------------------------------------------------------------------------
# Senses
# ---------------------------------------------------------------------------

    def want_to_rest(self):
        """Returns if the agent wants to rest.      
        """
        if self.resting_time > 0:      
            self.resting_time -= 1             # reduces resting time by one, if agent is resting  
        else:
            if self.random.random() <= 0.5:    
                self.resting_status = 1        # with a chance of 50% each agent is sent to rest, if it is not resting
            else:
                self.resting_status = 0
            self.resting_time = self.random.randint(10, 30) # set resting to a value between 10 and 30 
        return self.resting_status