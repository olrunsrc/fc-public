from __future__ import nested_scopes

# for POSH.Behaviour
import POSH


# global variables for the behavior grooming

GROOM_MAX_VALUE = 100 
GROOM_MIN_VALUE = 30

# avoid behaviour from being loaded (rather use the one from Grooming.py)
#class Grooming(POSH.Behaviour):

    def __init__(self, agent):                       # initialises grooming behavior
        
        POSH.Behaviour.__init__(self, agent,        # registers its actions and senses 
            ('pick_groom_target', 'move_to_groom_target', 'pick_new_target',
             'groom_with_target', 'increase_groom_desire_time'),
            ('wants_to_groom', 'has_groom_target',
             'is_near_groom_target', 'groom_target_wants_to_groom'))
        
        # variables for the behavior
        self.groom_target = None                           # groom targets  
        self.groom_desire = self.random.randint(30, 70)    # desire to groom, as lower the value as higher the desire  
        self.after_grooming_latch = 1                      # latch for whether I have groomed before
        
        # Set the inspectors
        self.registerInspectors(('GroomTarget', 'GroomStatus', 'GroomDesire'))

# ---------------------------------------------------------------------------
# This defines the inspectors
# ---------------------------------------------------------------------------
    
    def getGroomDesire(self):
        return int(self.groom_desire)
    
    def setGroomDesire(self, desire):
        self.groom_desire = float(level)
    
    
    def getGroomTarget(self):
        if self.groom_target:
            return self.groom_target.getAgentName()
        else:
            return "None"
    
    
    def getGroomStatus(self):
        return self.after_grooming_latch
    
    def setGroomStatus(self, status):
        self.after_grooming_latch = int(status)

   
# ---------------------------------------------------------------------------
# Actions
# ---------------------------------------------------------------------------

    def pick_groom_target(self):            # picks the closest agent as groom target        
        #original so      closestAgentself.groom_target = self.agent.MASON.closest('agents')
        self.groom_target = None
        ignoreList = []
        anzahlVersuche = 1
            
        while self.groom_target == None and anzahlVersuche > 0:
            closestAgent = self.agent.MASON.closest('agents', ignoreList)
            if closestAgent == None:
                print('ran out of possible groom targets!')
                break
            elif not closestAgent.agent.Grooming.wants_to_groom():
                # print('ignoring: ', anzahlVersuche, closestAgent)
                ignoreList.append(closestAgent)
                anzahlVersuche = anzahlVersuche - 1
            else:
                print('new groom target: ', closestAgent.getAgentName())
                self.groom_target = closestAgent
  
    def pick_new_target(self):
        if self.groom_target != None:            
            # I think this should pick a new agent (if we have already one) but excludes the last target from choice.
            self.groom_target = \
                self.agent.MASON.closest('agents', (self.groom_target,))
   
    
    def move_to_groom_target(self):            # moves towards target, taking a step size of 3        
        self.agent.MASON.moveTo(self.groom_target, 3)
        
    
    def groom_with_target(self):               # increase groom value 1 unit per time, if target wants to groom. Stops at max value.
        self.groom_target.groom_target_wants_to_groom = 1
        self.groom_desire -= 1
        if self.groom_desire <= GROOM_MIN_VALUE:
            return 0
                    
        
    def increase_groom_desire_time(self):      # increases the groom desire by 0.5 units per time step
        self.groom_desire += 0.5
        return 1
    
# ---------------------------------------------------------------------------
# Senses
# ---------------------------------------------------------------------------
    
    def wants_to_groom(self):                  # returns whether the agent wants to groom or get groomed.
        if self.groom_desire <= GROOM_MIN_VALUE:
            self.after_grooming_latch = 1
            self.groom_target = None
            return 0
        if self.after_grooming_latch:
            if self.groom_desire < GROOM_MAX_VALUE:
                return 0
            else:
                self.after_grooming_latch = 0
                return 1
        else:
            return 1
    
    
    def has_groom_target(self):                 # returns if the agent has a groom target
        return (self.groom_target != None)
    
    
    def is_near_groom_target(self):             # Returns if the agent is within a distance of 3 of its current groom target.   
        if self.groom_target and \
           self.agent.MASON.distance(self.groom_target) < 3:
            return 1
        return 0
    
    
    def groom_target_wants_to_groom(self):      # returns if the groom target want to groom
        if self.groom_target and \
           self.groom_target.agent.Grooming.wants_to_groom():
            return 1
        return 0
