"""Implementation of DriveCollection, DrivePriorityElement and DriveElement.

A L{POSH.strict.DriveCollection} contains several
L{POSH.strict.DrivePriorityElement}s
that contains several L{POSH.strict.DriveElement}s. Upon firing a drive
collection, either the goal is satisfied, or either of the drive
priority elements needs to be fired successfully. Otherwise, the
drive fails. The drive priority elements are tested in order or
their priority. A drive priority element fires successfully if one
of its drive elements is ready and can be fired.
"""

# Jython compatibility
from __future__ import nested_scopes
from POSH.jython_compat import *

# POSH modules
from element import Element, ElementCollection, FireResult
from action import Action
import random

class DriveCollection(ElementCollection):
    """A drive collection, containing drive priority elements.
    """
    def __init__(self, agent, collection_name, priority_elements, goal):
        """Initialises the drive collection.
        
        The log domain is set to [AgentId].DC.[collection_name]

        If no goal is given (goal = None), then it can never be satisfied.
        
        @param agent: The collection's agent.
        @type agent: L{POSH.strict.Agent}
        @param collection_name: The name of the drive collection.
        @type collection_name: string
        @param priority_elements: The drive elements in order of their
            priority, starting with the highest priority.
        @type priority_elements: sequence of L{POSH.strict.DrivePriorityElement}
        @param goal: The goal of the drive collection.
        @type goal: L{POSH.strict.Trigger} or None
        """
        ElementCollection.__init__(self, agent, "DC.%s" % collection_name)
        self._name = collection_name
        self._elements = priority_elements
        self._goal = goal
        self.log.debug("Created")
        
        self.last_triggered_element=None
    
    def reset(self):
        """Resets all the priority elements of the drive collection.
        """
        self.log.debug("Reset")
        for element in self._elements:
            element.reset()
    
    def fire(self):
        """Fires the drive collection.
        
        This method first checks if the goal (if not None) is met. If
        that is the case, then FireResult(False, self) is
        returned. Otherwise it goes through the list of priority
        elements until the first one was fired successfully (returning
        something else than None). In that case, FireResult(True,
        None) is returned. If none of the priority elements were
        successful, FireResult(False, None) is returned, indicating a
        failing of the drive collection.
        
        To summarise:
            - FireResult(True, None): drive element fired
            - FireResult(False, self): goal reached
            - FireResult(False, None): drive failed
        
        @return: The result of firing the drive.
        @rtype: L{POSH.strict.FireResult}
        """
        self.log.debug("Fired")
        # check if goal reached
        if self._goal and self._goal.fire():
            self.log.debug("Goal Satisfied")
            return FireResult(False, self)
        # fire elements
        for element in self._elements:
            # a priority element returns None if it wasn't
            # successfully fired
            if element.fire() != None:
                return FireResult(True, None)
        # drive failed (no element fired)
        self.log.debug("Failed")
        return FireResult(False, None)
    
    def copy(self):
        """Is never supposed to be called and raises an error.
        
        @raise NotImplementedError: always
        """
        raise NotImplementedError, \
            "DriveCollection.copy() is never supposed to be called"


class DrivePriorityElement(ElementCollection):
    """A drive priority element, containing drive elements.
    """
    def __init__(self, agent, drive_name, elements):
        """Initialises the drive priority element.
        
        The log domain is set to [AgentName].DP.[drive_name]
        
        @param agent: The element's agent.
        @type agent: L{POSH.strict.Agent}
        @param drive_name: The name of the associated drive.
        @type drive_name: string
        @param elements: The drive elements of the priority element.
        @type elements: sequence of L{POSH.strict.DriveElement}
        """
        ElementCollection.__init__(self, agent, "DP.%s" % drive_name)
        self._name = drive_name
        self._elements = elements
        self._timer = agent.getTimer()
        self.log.debug("Created")
        
        self.agent=agent
    
    def reset(self):
        """Resets all drive elements in the priority element.
        """
        self.log.debug("Reset")
        for element in self._elements:
            element.reset()
    
    def fire(self):
        """Fires the drive prority element.
        
        This method fires the first ready drive element in its
        list and returns FireResult(False, None). If no
        drive element was ready, then None is returned.
        
        @return: The result of firing the element.
        @rtype: L{POSH.strict.FireResult} or None
        """
        self.log.debug("Fired")
        timestamp = self._timer.time()
        
        random.shuffle(self._elements)  
        #new_elements=self.get_sorted_drive()      
        
        if self.agent._dc.last_triggered_element in self._elements:
            if self.agent._dc.last_triggered_element.isReady(timestamp):                
#                if not self.agent._dc.last_triggered_element._behaviours[0].wants_to_interrupt():
#                    for element in new_elements:
#                        if element.isReady(timestamp) and element._behaviours[0].wants_to_interrupt():#and element!=self.agent._dc.last_triggered_element
#                            self.agent._dc.last_triggered_element=element
#                            element.fire()
#                            return FireResult(False, None)                                        
                self.agent._dc.last_triggered_element.fire()
                return FireResult(False, None)
        
        #for element in new_elements:
        for element in self._elements:            
            if element.isReady(timestamp):
                if element != self.agent._dc.last_triggered_element:
                    if self.agent._dc.last_triggered_element==None:
                        if element.is_latched:
                            self.agent._dc.last_triggered_element=element
                    else:                 
                        if not self.agent._dc.last_triggered_element.isReady(timestamp):#event finished naturally
                            self.agent._dc.last_triggered_element=None
                        else:                    
                            behaviours=self.agent._dc.last_triggered_element._behaviours
                                                        
                            for b in behaviours:
                                b.signal_interrupt()
                                                        
                            if element.is_latched:
                                self.agent._dc.last_triggered_element=element
                            else:
                                self.agent._dc.last_triggered_element=None
                                                  
                element.fire()
                return FireResult(False, None)

        return None

    def get_sorted_drive(self):
        all_elements=[]
        for index,element in enumerate(self._elements):
            if element.is_latched:
                all_elements.append((element._behaviours[0].get_urgency(),index))
            else:
                all_elements.append((0,index))
        all_elements.sort()
        new_elements=[]
        for pair in all_elements:
            new_elements.append(self._elements[pair[1]])
        return new_elements  

    def copy(self):
        """Is never supposed to be called and raises an error.
        
        @raise NotImplementedError: always
        """
        raise NotImplementedError, \
            "DrivePriorityElement.copy() is never supposed to be called"

class DriveElement(Element):
    """A drive element.
    """
    def __init__(self, agent, element_name, trigger, root, max_freq):
        """Initialises the drive element.
        
        The log domain is set to [AgentName].DE.[element_name]
        
        @param agent: The element's agent.
        @type agent: L{POSH.strict.Agent}
        @param element_name: The name of the drive element.
        @type element_name: string
        @param trigger: The trigger of the element.
        @type trigger: L{POSH.strict.Trigger}
        @param root: The element's root element.
        @type root: L{POSH.strict.Action}, L{POSH.strict.Competence} or
            L{POSH.strict.ActionPattern}
        @param max_freq: The maximum frequency at which is element is
            fired. The frequency is given in milliseconds between
            invocation. A negative number disables this feature.
        @type max_freq: long
        """
        Element.__init__(self, agent, "DE.%s" % element_name)
        self._name = element_name
        self._trigger = trigger
        self._root, self._element = root, root
        self._max_freq = max_freq
        # the timestamp when it was last fired
        self._last_fired = -100000l
        self.log.debug("Created")
        
        self.agent=agent
        
        self.is_latched=False        
        self._behaviours=[]
        
        for sense in self._trigger._senses:            
            self._behaviours.append(sense.behaviour)        
    
    def reset(self):
        """Resets the drive element to its root element,
        and resets the firing frequency.
        """
        self.log.debug("Reset")
        self._element = self._root
        self._last_fired = -100000l
    
    def isReady(self, timestamp):
        """Returns if the element is ready to be fired.
        
        The element is ready to be fired if its trigger is
        satisfied and if the time since the last firing is
        larger than the one given by C{maxFreq}. The time of the
        last firing is determined by the timestamp given
        to L{isReady} when it was called the last time and returned
        True. This implies that the element has to be fired
        every time when this method returns True.
        
        @param timestamp: The current timestamp in milliseconds
        @type timestamp: long.
        """
        if self._trigger.fire():
            if self._max_freq < 0 or \
               (timestamp - self._last_fired) >= self._max_freq:
                self._last_fired = timestamp
                return True
            else:
                self.log.debug("Max. firing frequency exceeded")
        return False
    
    def fire(self):
        """Fires the drive element.
        
        This method fires the current drive element and always
        returns None. It uses the slip-stack architecture to determine
        the element to fire in the next step.
        
        @return: None.
        @rtype: None
        """
        self.log.debug("Fired")
        element = self._element
        # if our element is an action, we just fire it and do
        # nothing afterwards. That's because we can only have an action
        # as an element, if it is the drive element's root element.
        # Hence, we didn't descend in the plan tree and can keep
        # the same element.
        # type() doesn't return the right thing, we need to use __class__
        if element.__class__ == Action:
            element.fire()
            self._element = self._root
            return None
        # the element is a competence or an action pattern
        result = element.fire()
        if result.continueExecution():
            # if we have a new next element, store it as the next
            # element to execute
            nextElement = result.nextElement()
            if nextElement:
                self._element = nextElement
        else:
            # we were told not to continue the execution -> back to root
            # We must not call reset() here, as that would also reset
            # the firing frequency of the element.
            self._element = self._root
        return None

    def copy(self):
        """Is never supposed to be called and raises an error.
        
        @raise NotImplementedError: always
        """
        raise NotImplementedError, \
            "DriveElement.copy() is never supposed to be called"
