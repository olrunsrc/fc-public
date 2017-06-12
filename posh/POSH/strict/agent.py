"""Implementation of a POSH Agent.
"""

# Jython compatibility
from __future__ import nested_scopes
from POSH.jython_compat import *

# Python modules
import thread, time

# POSH modules
from POSH.agent_base import AgentBase
from lapparser import LAPParser

# drive collection results
DRIVE_FOLLOWED = 0
DRIVE_WON = 1
DRIVE_LOST = -1


class Agent(AgentBase):
    """A strict POSH Agent.
    """
    def __init__(self, library, plan, attributes, world = None):
        # load behaviours and plan, and create tree
        AgentBase.__init__(self, library, plan, attributes, world)
        # set the initial loop frequency to 20Hz
        self.setLoopFreq(1000 / 20)
    
    def setTimer(self, timer):
        """Sets the agent timer.

        The agent timer determines the timing behaviour of an agent.
        Is is usually set when loading a plan, as the drive collection
        specifies if a stepped timer (DC) or a real-time timer (RDC) is
        required.

        @param timer: The agent's timer.
        @type timer: L{POSH.strict.TimerBase}
        """
        self._timer = timer

    def getTimer(self):
        """Returns the currently used timer.

        @return: The currently used timer.
        @rtype: L{POSH.strict.TimerBase}
        """
        return self._timer

    def setLoopFreq(self, freq):
        """Sets the loop frequency of real-time plans.

        Calling this method sets the loop frequency of real-time plans.
        The loop frequency is the frequency at which the main POSH loop
        is executed. The given frequency is an upper bound on the real
        execution frequency.

        @param freq: The loop frequency, given in milliseconds that
            pass between two calls of the main loop.
        @type freq: long
        """
        # causes an AttributeError for non-real-time timers
        self._timer.setLoopFreq(freq)

    def reset(self, waittime = 300):
        """Checks if the behaviours are ready and resets the agent's timer.

        This method should be called just before running the main loop.
        
        The waittime is the time allowed until the behaviours are getting
        ready. It is the same as given to checkError(). By default, it is
        set to 20 seconds.
        
        @param waittime: Timout waiting for behaviours (see L{checkError()}).
        @type waittime: int
        @return: If the reset was successful.
        @rtype: bool
        """
        if not AgentBase.reset(self):
            return False
        self._timer.reset()
        return True
    
    def followDrive(self):
        """Performes one loop through the drive collection.
        
        This method takes the first triggering drive element and either
        descends further down in the competence tree, or performs
        the drive's current action.
        
        It returns either DRIVE_WON if the drive collection's goal was
        reached, DRIVE_LOST if no drive triggered, or DRIVE_FOLLOWED if
        the goal wasn't reached and a drive triggered.
        
        @return: The result of processing the drive collection.
        @rtype: DRIVE_FOLLOWED, DRIVE_WON or DRIVE_LOST
        """
        
        """FIXME: This test is *very* costly, this function is the most frequently run in a POSH.  
        In lisp, I used to have a debug version of POSH that had lots of conditionals in it, 
        and a fast version with none.  S
        peaking of None, identity is faster to check than equality, according to python.org
        Maybe profile.py should replace the function maned followDrive... note this would require 
        knowing if your posh was strict or scheduled.
        JJB 1 Mar 08
        """     
        if (self.profiler is not None):
            self.profiler.increase_total_calls()
        
        self.log.debug("Processing Drive Collection")
        result = self._dc.fire()
        self._timer.loopEnd()
        if result.continueExecution():
            return DRIVE_FOLLOWED
        else:
            if result.nextElement():
                return DRIVE_WON
            else:
                return DRIVE_LOST

    def _loop_thread(self):
        """The loop thread, started by L{startLoop}.

        This thread controls how L{followDrive} is called.
        """
        while self.checkError(0) == 0:
            # check for pause
            if self._loop_pause:
                # check ever 10th of a second
                while self._loop_pause:
                    time.sleep(0.1)
                self._timer.reset()
            # check if stopLoop was called
            if not self._exec_loop:
                return
            # follow drive, and control the loop timing after that
            result = self.followDrive()
            if result in (DRIVE_WON, DRIVE_LOST):
                return
            self._timer.loopWait()

    def _loadPlan(self, planfile):
        """Loads the plan and creates the drive collection tree.
        
        The method parses the plan file, and then uses the plan builder to
        build the drive collection tree.
        
        @param planfile: Filename of the plan file that is loaded.
        @type planfile: string
        """
        # if setTimer() is not called, then the first use of
        # the timer will fail. setTimer() is called when the drive
        # collection is built.
        self._timer = None
        # read plan, parse it and build drive collection
        plan_str = open(planfile).read()
        plan_builder = LAPParser().parse(plan_str)
        self._dc = plan_builder.build(self)
