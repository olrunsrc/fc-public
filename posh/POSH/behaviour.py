"""Implementation of the Behaviour base class.
"""

# Jython compatibility
from __future__ import nested_scopes
from jython_compat import *

# POSH modules
from logbase import LogBase

class Behaviour(LogBase):
    """Behaviour base class.
    """
    # attribute names that cannot be overridden
    _reserved_attributes = ('agent', 'random',
                            '_actions', '_senses', '_inspectors')

    def __init__(self,agent,actions,senses,attributes=None,caller=None):
        """Initialises behaviour with given actions and senses.
        
        The actions and senses has to correspond to
          - the method names that implement those actions/senses
          - the names used in the plan
        
        The log domain of a behaviour is set to
        [AgentId].Behaviour
        
        
        @param agent: The agent that uses the behaviour
        @type agent: L{POSH.AgentBase}
        @param actions: The action names to register.
        @type actions: sequence of strings
        @param senses: The sense names to register.
        @type senses: sequence of string
        @param attributes: List of attributes to initialise behaviour state.
        @type attributes: as for L{POSH.Behaviour.assignAttributes}
        """
        LogBase.__init__(self, agent, "Behaviour")
        self.agent = agent
        # aquire the random number generator from the agent
        self.random = agent.random
                
        if caller!=None:
            self.get_actions_senses(caller)
        else:
            self._actions = actions
            self._senses = senses

        self._inspectors = None
        # assign attributes
        if attributes != None:
            self.assignAttributes(attributes)
            
    def get_actions_senses(self,source):
        import inspect,re
        all_defs=inspect.getmembers(source,inspect.isroutine)
        
        senses={}
        actions={}    
        
        s=re.compile('^s_[.]*')
        a=re.compile('^a_[.]*')
    
        for (name,definition) in all_defs:
            if s.match(name):
                senses[name]=getattr(source,name)
            if a.match(name):
                actions[name]=getattr(source,name)    
    
        self._actions=tuple(actions.keys())
        self._senses=tuple(senses.keys())              

    def getName(self):
        """Returns the name of the behaviour.
        
        The name of a behaviour is the same as the name of
        the class that implements it.
        
        @return: Name of the behaviour.
        @rtype: string
        """
        return self.__class__.__name__
    
    def getActions(self):
        """Returns a list of available actions.
        
        @return: List of behaviour actions.
        @rtype: sequence of strings
        """
        return self._actions
    
    def getSenses(self):
        """Returns a list of available senses.
        
        @return: List of behaviour senses.
        @rtype: sequence of strings
        """
        return self._senses
    
    def setRNG(self, rng):
        """Sets the random number generator of the behaviour.
        
        This method is called whenever the random number generator
        of the agent is changed. The random number generator is
        accessible through the behaviour attribute 'random'.__abs__
        
        @param rng: A random number generator.
        @type rng: Similar to python 'random' module
        """
        self.random = rng
    
    def assignAttributes(self, attributes):
        """Assigns the behaviour a set of attributes.
        
        The attributes are given by a dictionary attribute_name -> value.
        If the behaviour object already has an attribute with the given name,
        this attribute is only reassigned if it is not callable (e.g. a
        method) or a reserved attribute (see Behaviour._reserved_attributes
        for a list).
        
        @param attributes: dictionary of attributes to assign to behaviour.
        @type attributes: dictionary attribute_name -> value
        """
        for attr_name, value in attributes.items():
            if (hasattr(self, attr_name) and callable(getattr(self, attr_name))) or \
               attr_name in Behaviour._reserved_attributes:
                continue
            setattr(self, attr_name, value)

    def checkError(self):
        """Returns if the behaviour is ok.
        
        This method is called to make sure that the behaviour is ok at every
        cycle. In its default implementation it always returns False.
        
        @return: False for OK, True for not OK.
        @rtype: False or True
        """
        return False
    
    def exitPrepare(self):
        """Called by the agent upon a request for exit.
        
        This method prepares the behaviour to stop. In its default
        implementation it does nothing.
        """
        pass
    
    def reset(self):
        """Called by the agent before the main loop is started.
        
        This is the best place to connect the behaviours to the world, if
        required, or register them there. If, for example, a behaviour
        needs to establish a network connection, then this is the best place
        to establish this connection.
        
        The method has to return if it was successful or not. Alternatively, it
        can always return True, and then report the behaviour's state in
        checkError() which is usualy called after all behaviours have been
        resetted.
        
        In its default implementation, this method returns True.
        
        @return: If the reset was successful.
        @rtype: bool
        """
        return True
    
    def registerInspectors(self, inspectors):
        """Sets the methods to call to get/modify the state of the behaviour.
        
        Inspectors can be used to observe and modify the state of a behaviour.
        Each inspector has to be given by a string that gives the name of
        the behaviour method to be called. The method has to be named 'get'
        followed by the name of the inspector, and has to take 0 arguments
        (other than C{self}, naturally). If you want to allow changing
        the behaviour state, you have to provide another method taking a single
        string as an argument (besides the obligatory C{self}), and being
        called 'set' followed by the name of the inspector.
        
        Given, for example, that we want to control the energy level of a
        behaviour. Then, if the string 'Energy' is given to the
        inspector, it looks for the method 'getEnergy' to get the energy level.
        If another method 'setEnergy' is provided, taking a string as an
        argument, we can also modify the energy level of the behaviour.
        
        @param inspectors: A list of inspector methods, as described above.
        @type inspectors: sequence of strings.
        @raise AttributeError: If the inspector method cannot be found.
        """
        self._inspectors = []
        accessor, mutator = None, None
        for inspector in inspectors:
            accessor = getattr(self, "get%s" % inspector, None)
            mutator = getattr(self, "set%s" % inspector, None)
            if not accessor:
                raise AttributeError, "Could not find inspector method %s " \
                    "in behaviour %s" % (inspector, self._name)
            self._inspectors.append((inspector, accessor, mutator))

    def getInspectors(self):
        """Returns the list of currently registered inspectors.
        
        The list of inspectors contains elements of the form
        C{(name, accessor, mutator)}, where C{name} is the name of the
        inspector, C{accessor} is the accessor method (taking no arguments),
        and C{mutator} is the mutator method (taking a single string as its
        only argument), or C{None} if no mutator is provided.
        
        @return: List of inspectors.
        @rtype: Sequence of (string, method, method|None)
        """
        return self._inspectors
