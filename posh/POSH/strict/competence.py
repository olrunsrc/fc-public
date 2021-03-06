"""Implementation of Competence, CompetencePriorityElement and
CompetenceElement.

A L{Competence} contains a list of L{CompetencePriorityElement}s that
each contain some L{CompetenceElement}s. Upon firing a competence,
the competence finds the first element in the competence priority list
that executes successfully. A competence priority list executes
successfully if at least one of its elements is ready to fire and is
fired.
"""

# Jython compatibility
from __future__ import nested_scopes
from POSH.jython_compat import *

# POSH modules
from element import Element, ElementCollection, FireResult
from action import Action
from copy import copy

class Competence(ElementCollection):
    """A POSH competence, containing competence priority elements.
    """
    def __init__(self, agent, competence_name, priority_elements, goal):
        """Initialises the competence.

        If no goal is given, then the goal will never be reached.

        The log domain is set to "[AgentId].C.[competence_name]".
        
        @param agent: The competence's agent.
        @type agent: L{POSH.strict.Agent}
        @param competence_name: The name of the competence.
        @type competence_name: string
        @param priority_elements: The priority elements of the competence,
            in their order of priority.
        @type priority_elements: sequence of L{POSH.strict.CompetencePriorityElement}
        @param goal: The goal of the competence.
        @type goal: L{POSH.strict.Trigger} or None
        """
        ElementCollection.__init__(self, agent, "C.%s" % competence_name)
        self._name = competence_name
        self._elements = priority_elements
        self._goal = goal
        self.log.debug("Created")
    
    def reset(self):
        """Resets all the competence's priority elements.
        """
        self.log.debug("Reset")
        for element in self._elements:
            element.reset()
    
    def fire(self):
        """Fires the competence.
        
        This method first checks if the competence's goal is satisfied
        (if the goal is not None). If that is the case, then it
        returns FireResult(False, None). Otherwise it fires the
        priority elements one by one. On the first successful firing
        of a competence priority element, the method returns the
        result of the priority element. If no priority element fired
        successfully, then FireResult(False, None) is returned.
        
        @return: The result of firing an element, or
            FireResult(False, None)
        @rtype: L{POSH.strict.FireResult}
        """
        self.log.debug("Fired")
        # check if goal is satisfied
        if self._goal and self._goal.fire():
            self.log.debug("Goal satisfied")
            return FireResult(False, None)
        # process the elements
        for element in self._elements:
            result = element.fire()
            # check if the competence priority element failed
            if result.continueExecution() and not result.nextElement():
                continue
            return result
        # we failed
        self.log.debug("Failed")
        return FireResult(False, None)
    
    def copy(self):
        """Returns a reset copy of itsself.
        
        This method creates a copy of itsself that has a copy of the
        competence priority elements but is otherwise equal.
        
        @return: A reset copy of itself.
        @rtype: L{POSH.strict.Competence}
        """
        # name and goal stays the same, only elements need to be copied
        # therefore we'll make a shallow copy of the object and
        # copy the elements separately
        new_obj = copy(self)
        new_elements = []
        for element in self._elements:
            new_elements.append(element.copy())
        new_obj._elements = new_elements
        return new_obj

    def setElements(self, elements):
        """Sets the list of priority elements of the competence.

        Calling this method also resets the competence.

        @param elements: The list of priority elements.
        @type elements: Sequence of L{POSH.strict.CompetencePriorityElement}
        """
        self._elements = elements
        self.reset()


class CompetencePriorityElement(ElementCollection):
    """A competence priority element, containing competence elements.
    """
    def __init__(self, agent, competence_name, elements):
        """Initialises the competence priority element.
        
        The log domain is set to [AgentName].CP.[competence_name]
        
        @param agent: The element's agent.
        @type agent: L{POSH.strict.Agent}
        @param competence_name: The name of the competence.
        @type competence_name: string
        @param elements: The set of competence elements of the
            priority element.
        @type elements: sequence of L{POSH.strict.CompetenceElement}
        """
        ElementCollection.__init__(self, agent, "CP.%s" % competence_name)
        self._name = competence_name
        self._elements = elements
        self.log.debug("Created")
    
    def reset(self):
        """Resets all its competence elements.
        """
        self.log.debug("Reset")
        for element in self._elements:
            element.reset()
    
    def fire(self):
        """Fires the competence priority element.
        
        This method goes through its list of competence elements
        and fires the first one that is ready. In that case,
        the result of the competence element is returned. Otherwise,
        it returns FireResult(True, None) (this can never be returned
        by a competence element and is therefore uniquely identifyable).
        
        @return: The result of firing the competence priority element.
        @rtype: L{POSH.strict.FireResult}
        """
        self.log.debug("Fired")
        for element in self._elements:
            # as the method ignores the timestamp, we can give it
            # whatever we want
            if element.isReady(0):
                return element.fire()
        self.log.debug("Priority Element failed")
        return FireResult(True, None)
    
    def copy(self):
        """Returns a reset copy of itsself.
        
        This method creates a copy of itsself that has a copy of the
        reset priority elements but is otherwise equal.
        
        @return: A reset copy of itself.
        @rtype: L{POSH.strict.CompetencePriorityElement}
        """
        # everything besides the elements stays the same. That's why
        # we make a shallow copy and only copy the elements separately.
        new_obj = copy(self)
        new_elements = []
        for element in self._elements:
            new_elements.append(element.copy())
        new_obj._elements = new_elements
        return new_obj


class CompetenceElement(Element):
    """A competence element.
    """
    def __init__(self, agent, element_name, trigger, element, max_retries):
        """Initialises the competence element.
        
        The log domain is set to [AgentName].CE.[element_name].
        
        @param agent: The competence element's agent.
        @type agent: L{POSH.strict.Agent}
        @param element_name: The name of the competence element.
        @type element_name: string
        @param trigger: The element's trigger
        @type trigger: L{POSH.strict.Trigger}
        @param element: The element to fire.
        @type element: L{POSH.strict.Action}, L{POSH.strict.Competence}, or
            L{POSH.strict.ActionPattern}
        @param max_retries: The maximum number of retires. If this is set
            to a negative number, it is ignored.
        @type max_retries: int
        """
        Element.__init__(self, agent, "CE.%s" % element_name)
        self._name = element_name
        self._trigger = trigger
        self._element = element
        self._max_retries = max_retries
        self._retries = 0
        self.log.debug("Created")
    
    def reset(self):
        """Resets the retry count.
        """
        self._retries = 0
    
    def isReady(self, timestamp):
        """Returns if the element is ready to be fired.
        
        The element is ready to be fired if its trigger is
        satisfied and it was not fired more than maxRetries.
        Note that C{timestamp} is ignored in this method. It is only
        there because L{isReady} is defined like that in the
        L{POSH.strict.Element} interface.
        
        @return: If the element is ready to be fired.
        @rtype: boolean
        """
        if self._trigger.fire():
            if self._max_retries < 0 or self._retries <= self._max_retries:
                self._retries += 1
                return True
            else:
                self.log.debug("Retry limit exceeded")
        return False
    
    def fire(self):
        """Fires the competence element.
        
        If the competence element's element is an Action, then this
        action is executed and FireResult(False, None) is returned.
        Otherwise, FireResult(True, element) is returned,
        indicating that at the next execution step that element has
        to be fired.
        
        @return: Result of firing the competence element.
        @rtype: L{POSH.strict.FireResult}
        """
        self.log.debug("Fired")
        element = self._element
        # as type() doesn't work, we have to use __class__
        if element.__class__ == Action:
            element.fire()
            return FireResult(False, None)
        return FireResult(True, element)
    
    def copy(self):
        """Returns a reset copy of itsself.
        
        This method creates a copy of itsself that links to the
        same element, but has a reset retry counter.
        
        @return: A reset copy of itself.
        @rtype: L{POSH.strict.CompetenceElement}
        """
        new_obj = copy(self)
        new_obj.reset()
        return new_obj
