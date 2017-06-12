from __future__ import nested_scopes

# for POSH.Behaviour
import POSH


GROOM_MAX_VALUE = 100 
GROOM_MIN_VALUE = 50

class Grooming(POSH.Behaviour):

    def __init__(self, agent):                       # initialises grooming behavior
        
        POSH.Behaviour.__init__(self, agent,        # registers its actions and senses 
            ('pick_groom_target', 'move_to_groom_target', 
             'groom_with_target', 'decrease_groom_saturation_time'),
            ('wants_to_groom', 'has_groom_target',
             'is_near_groom_target', 'groom_target_wants_to_groom'))
        
        # variables for the behavior
        self.groom_target = None                           # groom targets  
        self.groom_saturation = self.random.randint(30, 70)    # desire to groom, as lower the value as higher the desire  
        self.after_grooming_latch = 1                      # latch for whether I have groomed before
        
        # Set the inspectors
        self.registerInspectors(('GroomTarget', 'GroomStatus', 'GroomSaturation'))

# ---------------------------------------------------------------------------
# This defines the inspectors
# ---------------------------------------------------------------------------
    
    def getGroomSaturation(self):
        return int(self.groom_saturation)
    
    def setGroomSaturation(self, desire):
        self.groom_saturation = float(level)
    
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
        oldNeighbourhoodSize = self.agent.MASON.getNeighbourhoodSize()
        self.groom_target = None
        currentRadius =  100.0
        maxRadius     = 2000.0
        extendRadius  =  100.0
                    
        while self.groom_target == None and currentRadius <= maxRadius:
            # neighbours(<fields>, currentRadius) seems broken, MUST set NeighbourhoodSize manually
            self.agent.MASON.setNeighbourhoodSize(currentRadius)
            
            for element in self.agent.MASON.neighbours('agents', currentRadius):
                # 'Considering element %s at distance %s' % (element, self.agent.distance(element))
                if element.agent != self.agent and element.agent.Grooming.wants_to_groom():
                    print 'new groom target %s at distance %s' % (element, self.agent.MASON.distance(element))
                    self.groom_target = element
                    break
            currentRadius = currentRadius + extendRadius
        self.agent.MASON.setNeighbourhoodSize(oldNeighbourhoodSize)
         
                
    def move_to_groom_target(self):            # moves towards target, taking a step size of 3        
        self.agent.MASON.moveTo(self.groom_target, 3)
        
    
    def groom_with_target(self):               # increase groom value 1 unit per time, if target wants to groom. Stops at max value.
        self.groom_target.groom_target_wants_to_groom = 1
        self.groom_saturation += 8
        if self.groom_saturation <= GROOM_MIN_VALUE:
            return 0
                                      
        
    def decrease_groom_saturation_time(self):      # increases the groom desire by 0.5 units per time step
        self.groom_saturation -= 0.5
        return 1
    

# ---------------------------------------------------------------------------
# Senses
# ---------------------------------------------------------------------------
    
    def wants_to_groom(self):                  # returns whether the agent wants to groom or get groomed.
        if self.groom_saturation >= GROOM_MAX_VALUE:
            self.after_grooming_latch = 1
            self.groom_target = None
            return 0
        if self.after_grooming_latch:
            if self.groom_saturation > GROOM_MIN_VALUE:
                return 0
            else:
                self.after_grooming_latch = 0
                return 1
        else:
            return 1
    
    
    def has_groom_target(self):                 # returns if the agent has a groom target
        return self.groom_target != None
    
    
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

    
    
    
# def wants_to_groom(self):
#        """Returns if agent wants to groom or get groomed.
#        """
#        if self.groom_value >= GROOM_MAX_VALUE:
#            self.recently_groomed = 1
#            self.groom_target = None
#            return 0
#        if self.recently_groomed:
#            if self.groom_value > GROOM_MIN_VALUE:
#                return 0
#            else:
#                self.recently_groomed = 0
#                return 1
#        else:
#            return 1
#    
#    def has_groom_target(self):
#        """Returns if the agent has a groom target.
#        """
#        return not (self.groom_target == None)
#    
#    def is_near_groom_target(self):
#        """Returns if the agent is within a distance of 3 of its current
#        groom target.
#        """
#        if self.groom_target and \
#           self.agent.MASON.distance(self.groom_target) < 3:
#            return 1
#        return 0
#    
#def groom_target_wants_to_groom(self):
#        """Returns if the groom target want to groom.
#        """
#        if self.groom_target and \
#           self.groom_target.agent.Grooming.wants_to_groom():
#            return 1
#        return 0
