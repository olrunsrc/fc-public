# Jython compatibility
from __future__ import nested_scopes
from POSH.jython_compat import *

# for POSH.Behaviour
import POSH

# MASON 2d vector
from sim.util import Double2D

# some maths stuff
from math import sin, cos, atan2, pi

# maximum panic level
PANIC_MAX = 10
# standard panic decay per set
PANIC_DECAY = 0.5
# maximum 'allowed' distance to other agents
MAXDISTANCE = 70
# minimum distance after which we want to avoid the other agent
MINDISTANCE = 5

# sets the behavioour constants
COHESION = 1.0
AVERSION = 1.0
CONSISTENCY = 1.0
RANDOMNESS = 1.0
STEPSIZE = 1


class PanicFlocking(POSH.Behaviour):
    """The PanicFlocking Behaviour.
    """
    def __init__(self, agent):
        """Initialises the PanicFlocking Behaviour.
        """
        # register the actions and senses
        POSH.Behaviour.__init__(self, agent,
                                ('set_panic_to_max', 'reduce_panic_level','move_to_target', 
                                 'pick_target', 'panic_flock_move', 'flock_move'),
                                ('neighbor_panic', 'predator_close', 'alone', 
                                 'no_target'))
        # Initial behaviour state
        self.target = None
        self.alone_count = 0
        self.normal_jump = .7
        self.panic_level = 0
        # Set the inspectors
        self.registerInspectors(('PanicLevel', 'Target', 'AloneCount'))

    def reset(self):
        # this call was moved from __init__() to here, as we cannot be
        # sure in __init__ that the behaviour MASON is already initialised.
        self.agent.MASON.setNeighbourhoodSize(50)
        return True

# ---------------------------------------------------------------------------
# Accessors and Mutators
# ---------------------------------------------------------------------------
    
    def getPanicLevel(self):
        return self.panic_level
    
    def setPanicLevel(self, level):
        self.panic_level = float(level)
    
    def getTarget(self):
        if self.target:
            return self.target.getName()
        else:
            return "None"
    
    def getAloneCount(self):
        return self.alone_count
    
    def setAloneCount(self, count):
        self.alone_count = int(count)

# ---------------------------------------------------------------------------
# Actions
# ---------------------------------------------------------------------------

    def set_panic_to_max(self):
        """Sets the agent's panic level to the maximum level.
        """
        self.panic_level = PANIC_MAX
        return 1
   
    def reduce_panic_level(self):
        """Decays the panic level according to PANIC_DECAY,
        and limits it to 0 from below.
        """
        self.panic_level = max(self.panic_level - PANIC_DECAY, 0)
        return 1
   
    def move_to_target(self):
       """Moves one step closer to the target.
       """
       if self.target:
           self.agent.MASON.moveTo(self.target.getLoc(), STEPSIZE)
	   return 1

    def pick_target(self):
       """Picks the closest sheep as its target to move to.
       """
       # closest() without arguments returns the closest entitiy of its
       # own fields, which is in this case the 'sheep' field (in the SheepDog
       # simulation)
       self.target = self.agent.MASON.closest()
       return 1
   
    def flock_move(self):
        """Same as panic_flock_move but without the panic factor.
        """
        sheep = self.agent.MASON.neighbours('sheeps',)
        # the the different vectors
        cohe = self.cohesionVector(sheep)
        aver = self.aversionVector(sheep)
        cons = self.consistencyVector(sheep)
        mome = self.momentumVector()
        rand = self.randomVector()
       
        # sum them all up
        dx = COHESION * cohe.x + AVERSION * aver.x + \
             CONSISTENCY * cons.x + RANDOMNESS * rand.x + mome.x
        dy = COHESION * cohe.y + AVERSION * aver.x + \
             CONSISTENCY * cons.y + RANDOMNESS * rand.y + mome.y
        
        # normalise and move
        d = (dx * dx + dy * dy) ** 0.5 / STEPSIZE
        self.agent.moveBy(dx / d, dy / d)
        return 1  

    def panic_flock_move(self):
        """Moves the agent according to its panic level.
        
        The more the sheep are paniced, the more they show flocking
        behaviour.
        """
        # a factor in the range [0.15, 1.15] representing the normalised
        # panic level
        panic_factor = self.panic_level / PANIC_MAX
        # all objects within the given neighbourhood. The neighbourhood
        # size increases with the panic level
        sheep = self.agent.MASON.neighbours('sheeps',
                    self.agent.MASON.getNeighbourhoodSize() * \
					(panic_factor + 0.15))
        # the the different vectors
        cohe = self.cohesionVector(sheep)
        aver = self.aversionVector(sheep)
        cons = self.consistencyVector(sheep)
        mome = self.momentumVector()
        rand = self.randomVector()
        
        # sum up the vectors that give flocking behaviour
        fx = COHESION * cohe.x + AVERSION * aver.x + CONSISTENCY * cons.x + \
             RANDOMNESS * rand.x
        fy = COHESION * cohe.y + AVERSION * aver.y + CONSISTENCY * cons.y + \
             RANDOMNESS * rand.y
        # normalise that vector
        d = (fx * fx + fy * fy) ** 0.5
        if d > 0.0:
            fx, fy = fx / d, fy / d
        # mix the momentum vector with the random vector and normalise it
        mx, my = mome.x + RANDOMNESS * rand.x, mome.y + RANDOMNESS * rand.y
        d = (mx * mx + my * my) ** 0.5
        if d > 0.0:
            mx, my = mx / d, my / d
        # mix it with the momentum vector
        flocking, momentum = 0.1 + 0.5 * panic_factor, 0.9 - 0.5 * panic_factor
        dx, dy = fx * flocking + mx * momentum, fy * flocking + my * momentum
        # normalise the vector to the step-size and the panic_factor
        d = (dx * dx + dy * dy) ** 0.5 / ((0.2 + 0.8 * panic_factor) * STEPSIZE)
        # move it that direction
        self.agent.MASON.moveBy(dx / d, dy / d)
        return 1
   
    def aversionVector(self, neighbours):
        """Returns a vector that points away from the neighbours.
        
        This vector weights its neighbour by the inverse of the
        distance and sums up vectors that point away from them.
        Hence, the closer the neighbour, the more that vector
        points away from it.
       
        The vector is returned as a unit vector.
        """
        vx, vy = 0.0, 0.0
        a = self.agent.MASON
        # the size of the neigbourhood
        ns = a.getNeighbourhoodSize()
        for n in neighbours:
            d = a.distance(n)
            if d <= ns: # and d < MINDISTANCE:
                # add vector towards the entitiy, weighted by squared distance,
                # which is the unit vector weighted by 1 / distance ** 2
                d2 = (d / MINDISTANCE) ** 5
                w = a.vectorTowards(n.getLoc())
                vx -= w.x / d2
                vy -= w.y / d2
        d = (vx * vx + vy * vy) ** 0.5
        if neighbours:
            return Double2D(vx, vy)
        return Double2D(0.0, 0.0)
    
    def cohesionVector(self, neighbours):
        """Returns a vector that points towards the centre of
        the given neighbourhood.
        
        The vector is returned as a unit vector.
        """
        vx, vy = 0.0, 0.0
        a = self.agent.MASON
        ns = a.getNeighbourhoodSize()
        for n in neighbours:
            d = a.distance(n)
            if d <= ns:
                w = a.vectorTowards(n.getLoc())
                vx += w.x
                vy += w.y
        d = (vx * vx + vy * vy) ** 0.5
        if d > 0.0:
            return Double2D(vx / d, vy / d)
        return Double2D(0.0, 0.0)
    
    def momentumVector(self):
        """Returns a vector representing the current momentum of the
        agent.
        
        The momentum represents the current direction of the agent.
        """
        ori = self.agent.MASON.ori
        return Double2D(cos(ori), sin(ori))
    
    def consistencyVector(self, neighbours):
        """Returns a vector representing the average direction of the neighbourhood.
        
        The average direction of the neighbouhood is the average
        direction of all agents in the neighbourhood.
        
        The vecor is a vector of length 0.25
        """
        vx, vy = 0.0, 0.0
        if neighbours:
            for n in neighbours:
                vx += cos(n.ori)
                vy += sin(n.ori)
            d = (vx * vx + vy * vy) ** 0.5 / 0.25
            return Double2D(vx / d, vy / d)
        return Double2D(0.0, 0.0)
    
    def randomVector(self):
        """Returns a random vector of length 0.1.
        """
        random = self.agent.random.random
        x, y = random() - 0.5, random() - 0.5
        d = (x * x + y * y) ** 0.5 / 0.1
        # assuming that d > 0.0
        return Double2D(x / d, y / d)
        
# ---------------------------------------------------------------------------
# Senses
# ---------------------------------------------------------------------------
    def neighbor_panic(self):
        """Returns if the neighbours panic enough for onesself to
        start panicking.
        """
        # only if we have 1/4 panic maximum
        if self.panic_level < 0.25 * PANIC_MAX:
 #           for n in self.agent.MASON.neighbours('sheeps'):
            for n in self.agent.MASON.neighbours('sheeps', 40):
                # if any ot the neighbours is above 70% panic level
                if n.agent.PanicFlocking.panic_level > 0.70 * PANIC_MAX:
                    return 1
        return 0
 
    def predator_close(self):
        """Returns if there is a dog close.
        """
        return len(self.agent.MASON.neighbours(('dogs',))) > 0
 
    def alone(self):
        """Checks at every 50th call if the closest other agent is
        more than MAXDISTANCE away.
        """
        self.alone_count += 1
        if self.alone_count > 50:
            self.alone_count = 0
            if self.agent.MASON.distance(self.agent.MASON.closest()) > MAXDISTANCE:
                return 1
        return 0
 
    def no_target(self):
        """Returns if we have a target.
        """
        return (self.target == None)
 