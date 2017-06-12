from __future__ import nested_scopes
# for POSH.Behaviour
import POSH

FOOD_MAX_VALUE = 100
FOOD_MIN_VALUE = 50


class Feeding(POSH.Behaviour):
    
    def __init__(self, agent):                            # initialises the eating behaviour
      
        POSH.Behaviour.__init__(self, agent,             # registers its actions and senses
            ('pick_food', 'move_to_food', 'feed',         
             'decrease_food_saturation_time'),                       
            ('wants_food', 'is_near_food_target', 'has_food'))
                                                              
        
        self.food_target = None                               # food target 
        self.food_saturation = self.random.randint (50, 90)   # food_saturation, the smaller the number the more hungry 
        self.after_eating_latch = 1                           # latch for whether I have eaten before
        self.i_am_feeding = 0                                 # am I eating at the moment?
        self.empty_food = []                                  # creates a list of food that was found to be maxed out;
        self.exp = self.random.randint (0, 100)
        # sets inspectors
        self.registerInspectors(('FoodSaturation', 'Experience'))

# ---------------------------------------------------------------------------
# This defines the inspectors
# ---------------------------------------------------------------------------
    
    def getFoodSaturation(self):
        return int(self.food_saturation)
    
    def setFoodSaturation(self, level):
        self.food_saturation = float(level)

    def getExperience(self):
        return int(self.exp)
    
    def setExperience(self, level):
        self.exp = float(level)

# ---------------------------------------------------------------------------
# Actions
# ---------------------------------------------------------------------------
    
    def pick_food(self):                     # picks closest food target
        # save original neighbourhoodSize        
        oldNeighbourhoodSize = self.agent.MASON.getNeighbourhoodSize()
        self.food_target = None
        currentRadius =   50.0
        maxRadius     = 2000.0
        extendRadius  = 1000.0
                    
        while self.food_target == None and currentRadius <= maxRadius:
            # Hmmm... neighbours(<fields>, currentRadius) seems broken, MUST set NeighbourhoodSize manually
            self.agent.MASON.setNeighbourhoodSize(currentRadius)
            
            for element in self.agent.MASON.neighbours('food', currentRadius):
                # 'Considering element %s at distance %s' % (element, self.agent.MASON.distance(element))
                if not element.agent.Growing.tree_is_full():
                    # print 'new food target %s at distance %s' % (element, self.agent.MASON.distance(element))
                    self.food_target = element
                    self.i_am_feeding = 1
#                    element.reduce_capacity()
                    break
            currentRadius = currentRadius + extendRadius
            
        # restore original neighbourhoodSize
        self.agent.MASON.setNeighbourhoodSize(oldNeighbourhoodSize)
        
        # keep on starving... but move
        if self.i_am_feeding != 1:
             self.decrease_food_saturation_time()
             self.agent.Exploring.move_around()

                 
    
    def move_to_food(self):                  # Takes a step of 3 towards the current food target.
        self.agent.MASON.moveTo(self.food_target.loc, 3)
    
    
    def feed(self):                          # increases saturation until it stops
        self.food_saturation += 7       
#        if Befehl replace  
#        while feeding then pick_new_food_target 
        if self.food_saturation >= FOOD_MAX_VALUE:            
            print "freeing the resource, not hungry anymore", self.agent
#            self.food_target.increase_capacity()
            self.food_target = None
            self.i_am_feeding = 0
            return 0
         
    
    def decrease_food_saturation_time(self):          # decreases food_saturation 0.1 points every timesteps           
        self.food_saturation -= 1.0
        if self.food_saturation < 0 :
            print "whoops... I'm dead"
            # bear a new agent by "recycling" the old one
            self.agent.MASON.setLoc(self.agent.MASON.fieldCentre('food'))
            self.food_saturation = FOOD_MAX_VALUE  
            self.food_target = None       
            self.i_am_feeding = 0
        return 1
    
    
#    def comparison(self):                      # vergleicht experience werte der agenten
#        if abs(self.exp - other.exp) >= 40
#            if self.exp < other.exp)
#                pick_new_food_target
#            else 1 (gib Befehl replace zu dem anderen)
#        else 0 (fight) 
        

#    def fight(self):
#        set self.groom_saturation = 30        
#        if exp.self / (exp.self + exp.other) >= random(0;1)
#            exp.self + (x * abs(exp.self - exp.other)) and
#            exp.other - (x * abs(exp.self - exp.other))
#            replace zu dem anderen             
#         else exp.self - (x * abs(exp.self - exp.other)) and
#              exp.other + (x * abs(exp.self - exp.other))
#              pick_new_food_target          
#        --> hier muss gezaehlt und in eine Excel lesbare datei geschrieben werden


#        --> hier muss Befehl fuer die Heruntersetzung des groom desires bei schon
#            fressendem Agenten einfuegen
 

#    def replace
#        if --> hier muss weitergearbeitet werden
        
# ---------------------------------------------------------------------------
# Senses
# ---------------------------------------------------------------------------
    
    def wants_food(self):    
        # please keep me feeding while i'm at it
        if self.i_am_feeding == 1:
            return 1
        else:
            return self.food_saturation <= FOOD_MIN_VALUE

    def has_food(self):                   # returns if target is not none
        return (self.food_target != None)

    def is_near_food_target(self):                        # returns if the agent is within a distance of 2 of its current food target        
        if self.food_target and \
           self.agent.MASON.distance(self.food_target) < 2:     #die Zahl gibt den Radius UM den Futterpunkt an in dem der Agent futtern kann (!muss mindestens 2 sein!)
            return 1
        return 0
    
#    def free_food(self):                   # returns if anyone is on tree
#        if jeder but me sits on tree
#            return 0
#        return 1
    
#    def reduce_capacity(self):
#        self.agent_capacity -= 1
#        if self.agent_capacity < 0:
#            print "oops... sub- zero capacity after decreasing"
#            self.agent_capacity = 0
#
#    def increase_capacity(self):
#        self.agent_capacity += 1
#
#    def tree_is_full(self):
#        if self.agent_capacity <= 0:
#            return 1
#        return 0    