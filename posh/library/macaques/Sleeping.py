from __future__ import nested_scopes

# for POSH.Behaviour
import POSH

# dome more documentation would be nice
SLEEP_MAX_THRESH = 100
SLEEP_MIN_THRESH = 25

class Sleeping(POSH.Behaviour):
    """The Sleeping Behaviour.
    """
    def __init__(self, agent):
        """Initialise the sleeping behaviour.
        """
        # register actions and senses
        POSH.Behaviour.__init__(self, agent,
            ('pick_tree', 'move_to_tree', 'sleep_on_tree', 'reduce_sleep_threshold_time'),
            ('is_tired', 'has_tree'))
        # set the initial behaviour state
        # opens sleep target list
        self.sleep_target = None
        # desire to sleep
        self.sleep_threshold = 30 + self.random.uniform(0, 40)
        # have I just slept?
        self.recently_sleep = 0

# ---------------------------------------------------------------------------
# Actions
# ---------------------------------------------------------------------------

    def pick_tree(self):
        """Lets the agent pick the closes tree."""
        self.tree = self.agent.MASON.closest('shelter')
    
    def move_to_tree(self):
        """Moves the agent 2 units towards the picked tree."""
        self.MASON.agent.moveTo(self.tree, 2)
    
    def sleep_on_tree(self):
        """Lets the agent sleep and reduces its sleep threshold.
        """
        print "SLEEPING! %s, %i" % (str(self.sleep_threshold), int(self.recently_slept))
        # that seems to be wrong, but as I don't know the intnetions, I didn't change it
        self.sleep_threshold = self.sleep_threshold + 2
        if self.sleep_threshold > SLEEP_MAX_THRESH:
            self.sleep_threshold = SLEEP_MAX_THRESH
        # since when do trees have behaviours?
        # that will cause an error!!!
        self.tree.agent.Sleeping.sleep_threshold += 2    
    
    def reduce_sleep_threshold_time(self):
        """Reduces the sleep threshold by 0.2
        
        That means one unit reduction every 5 steps.
        """
        self.sleep_threshold = self.sleep_threshold - .2
    
# ---------------------------------------------------------------------------
# Senses
# ---------------------------------------------------------------------------

    def is_tired(self):
        """Returns if the agent is tried or not.
        """
        if self.sleep_threshold >= SLEEP_MAX_THRESH:
            self.recently_slept = 1
            return 0
        if self.recently_slept:
            if self.sleep_threshold > SLEEP_MIN_THRESH:
                return 0
            else:
                self.recently_slept = 0
                return 1
        else:
            return 1
    
    def has_tree(self):
        """Returns if the agent has picked a tree.
        """
        return (self.tree != None)