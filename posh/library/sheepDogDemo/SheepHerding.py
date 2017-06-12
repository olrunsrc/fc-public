from __future__ import nested_scopes

# behaviour base for POSH or SPOSH behaviours
from POSH import Behaviour

from sim.util import Double2D

# some maths stuff
from math import sin, cos, pi

from time import time

STEPSIZE = 1.3
MAXHERDWIDTH = 70
MINDISTANCE = 20
# the max number of seconds dogs will hold still while resting (assuming they got to their rest spot instantaneously)
OLDTIME = 6

class SheepHerding(Behaviour):
    """The SheepHerding Behaviour.
    """
    def __init__(self, agent):
        """Initialises the SheepHerding Behaviour.
        """
        self.succeed = lambda: 1
        Behaviour.__init__(self, agent,
                                ('set_rest_spot', 'select_sheep_target',
                                 'move_to_sheep_target', 'move_to_rest_spot',
                                 'rest', 'lose_target'),
                                ('herd_too_wide', 'has_target', 'has_rest_spot',
                                 'in_rest_spot', 'succeed', 'rest_spot_old'))
        # set the initial state
        self.restSpot = None
        self.restTime = 0
        self.target = None
        # register inspectors
        self.registerInspectors(('Target', 'RestSpot'))

# ---------------------------------------------------------------------------
# Accessors and Mutators
# ---------------------------------------------------------------------------

    def getTarget(self):
        if self.target:
            return self.target.getName()
        else:
            return "None"
    
    def getRestSpot(self):
        return self.restSpot

# ---------------------------------------------------------------------------
# Actions
# ---------------------------------------------------------------------------
        
    def set_rest_spot(self):
        """Sets the new resting spot, at RESTDISTANCE away from the
        herd centre.
        """
        a = self.agent.MASON
        f = a.fields[0]
        # get the center of the sheep field
        c = a.fieldCentre('sheeps')
        # get simulation width and height
        w, h = a.sim.width, a.sim.height
        # the new resting spot is at a random angle from the herd centre, 60% away
        ori = a.random.random() * 2 * pi
        rx = cos(ori) * w * 0.6
        ry = sin(ori) * h * 0.6
        self.restSpot = Double2D(f.stx(rx), f.sty(ry))
        self.restTime = time()
        self.target = None
	
    def select_sheep_target(self):
        """Selects the third-closest sheep as target.
        """
        a = self.agent.MASON
        # get the sheep
        sheep = a.sim.getField('sheeps').allObjects.objs
        # create a list of (obj, distance)
        sheepList = []
        for s in sheep:
            if s:
                sheepList.append((s, a.distance(s)))
        # sort the list by the distance
        sheepList.sort(lambda x, y: y[1] < x[1])
        # the target is the third-distant sheep
        sidx = min(3, len(sheepList)) - 1
        self.target = sheepList[sidx][0]
        self.restSpot = None
        return 1
	
    def lose_target(self):
        """Forget where you were going (presumably because you got there)
        """
        self.target = None
        return 1
    
    def move_to_sheep_target(self):
        """Take one step towards the target.  
        Note this is a deprecated behaviour that doesn't require sheep & adds the target loss / mindistance.
        """
        self.agent.MASON.moveTo(self.target, STEPSIZE)
        if self.agent.MASON.distance(self.target) < MINDISTANCE:
            self.target = None
        return 1
	
    def move_to_rest_spot(self):
        """Takes one step towards the resting spot.
        """
        # get the orientation and calculate a unit vector
        ori = self.agent.MASON.ori
        ox, oy = cos(ori), sin(ori)
        # get the vector towards the resting spot as a uni vector
        v = self.agent.MASON.vectorTowards(self.restSpot)
        d = (v.x * v.x + v.y * v.y) ** 0.5
        vx, vy = v.x / d, v.y / d
        # the new motion is with momentum
        vx, vy = vx + ox * 0.5, vy + oy * 0.5
        d = (vx * vx + vy * vy) ** 0.5 / STEPSIZE
        # move there
        self.agent.MASON.moveBy(vx / d, vy / d)
        return 1

    def rest(self):
        print "rest"

    
# ---------------------------------------------------------------------------
# Senses
# ---------------------------------------------------------------------------

    def herd_too_wide(self):
        """Return if the herd is too wide.
        """
        return (self.agent.MASON.fieldWidth('sheeps') > MAXHERDWIDTH)
	
    def has_target(self):
        """Returns if agent has target.
        """
        return (self.target != None)

    def has_rest_spot(self):
        """Returns if agent has resting spot.
        """
        return (self.restSpot != None)
	
    def in_rest_spot(self):
        """Returns if agent is in resting spot.
        """
        if self.restSpot:
            return (self.agent.MASON.distance(self.restSpot) < STEPSIZE)	
        return 0
    
    def rest_spot_old(self):
        return (time() > self.restTime + OLDTIME )
    
    
