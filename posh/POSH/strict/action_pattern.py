"""Implementation of an ActionPattern.
"""

# Jython compatibility
from __future__ import nested_scopes
from POSH.jython_compat import *

# Python modules
from copy import copy

# POSH modules
from element import ElementCollection, FireResult
from action import Action
from sense import Sense


class ActionPattern(ElementCollection):
    """An Action Pattern.
    """
    def __init__(self, agent, pattern_name, elements):
        """Initialises the action pattern.
        
        The log domain is set to [AgentId].AP.[patternName]
        
        @param agent: The corresponding agent.
        @type agent: L{POSH.strict.Agent}
        @param pattern_name: The name of the action pattern.
        @type pattern_name: string
        @param elements: The sequence of actions, with an optional
            competence as the final element.
        @type elements: sequence of L{POSH.strict.Action}, L{POSH.strict.Sense}
            and L{POSH.strict.Competence}
        """
        ElementCollection.__init__(self, agent, "AP.%s" % pattern_name)
        self._name = pattern_name
        self._elements = elements
        self._element_idx = 0
        self.log.debug("Created")
    
    def reset(self):
        """Resets the action pattern.
        
        This method sets the action pattern to fire the
        first action of the pattern upon the next call to L{fire}.
        """
        self.log.debug("Reset")
        self._element_idx = 0
    
    def fire(self):
        """Fires the action pattern.
        
        This method fires the current action / sense / sense-act or
        competence of the pattern. In case of firing an action / sense
        / sense-act, the method points to the next element in the
        pattern and returns FireResult(True, None) if the current
        action / sense / sense-act was successful (i.e. evaluated to
        True) and not the last action in the sequence, in which case
        it returns FireResult(False, None) and resets the action
        pattern.
        
        If the current element is a competence, then competence is
        returned as the next element by returning
        FireResult(True, competence), and the action pattern is
        reset.
        
        @return: The result of firing the action pattern.
        @rtype: L{POSH.strict.FireResult}
        """
        self.log.debug("Fired")
        element = self._elements[self._element_idx]
        # type() doesn't work, which is why we have to use __class__
        if element.__class__ == Action or element.__class__ == Sense:
            # check if action was successful
            if not element.fire():
                self.log.debug("Action/Sense '%s' failed" % element.getName())
                self._element_idx = 0
                return FireResult(False, None)
            # check if we've just fired the last action
            self._element_idx += 1
            if self._element_idx >= len(self._elements):
                self._element_idx = 0
                return FireResult(False, None)
            return FireResult(True, None)
        else:
            # we have a competence
            self._element_idx = 0
            return FireResult(True, element)
    
    def copy(self):
        """Returns a reset copy of itsself.
        
        This method returns a copy of itsself, and calls L{reset}
        on it.
        
        @return: A reset copy of itsself.
        @rtype: L{POSH.strict.ActionPattern}
        """
        new_obj = copy(self)
        new_obj.reset()
        return new_obj

    def setElements(self, elements):
        """Sets the elements of an action pattern.

        Calling this method also resets the action pattern.

        @param elements: The list of elements of the action patterns.
        @type elements: sequence of L{POSH.strict.Action} or 
            L{POSH.strict.Competence} (as last element of the sequence)
        """
        self._elements = elements
        self.reset()
