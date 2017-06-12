"""Launches a POSH agent or a set of agents.

Synopsis:
    launch.py [OPTIONS] library

Description:
    Launches a POSH agent by fist initialising the world and then the
    agents. The specified library is the behaviour library that will be used.
    
    -v, --verbose
        writes more initialisation information to the standard output.
    
    -h, --help
        print this help message.
    
    World initialisation:
    
    -w, --init-world-file=INITSCRIPT
        the python script that initialises the world. To communicate with
        launch.py, an instance of class World called 'world' is passed to the
        world initialisation script. Its most important methods:
            world.args() : Returns the arguments given by the -a options.
                If -a is not given, None is returned.
            world.set(x) : Passes x as the world object to the agents upon
                initialising them.
            world.createsAgents() : Needs to be called if the
                world initialisation script rather than launch.py creates
                and runs the agents.
        More information on the World class can be found in the API
        documenatation of the POSH.utils.World class.
        If no world initialisation script is specified, then the default world
        initialisation function of the library is called.
    
    -a, --init-world-args=ARGS
        the argument string given to the function init_world(args) in the
        script specified by -w. If no such script is given, the the arguments
        are given to the default world initialisation function of the library.
    
    Agent initialisation:
        If none of the below options are given, then the default library
        initialisation file is used to initialise the agent(s).
    
    -i, --init-agent-file=INITFILE
        initialises the agent(s) according to the given file. The file format
        is described below.
    
    -p, --plan-file=PLANFILE
        initialises a single agent to use the given plan. Only the name of
        the plan without the path needs to be given, as it is assumed to have
        the ending '.lap' and reside in the default location in the
        corresponding behaviour library. This option is only valid if -i 
        is not given.

Agent initialisation file format:
    The agent initialisation file allows the initialisation of one or several
    agents at once. The file is a simple text file that is read line by line.
    Each new agents starts with a '[plan]' line that specifyies the plan that
    the agent uses. This is followed by a list of attributes and values to
    initialise the behaviours of the agent. Empty lines, and lines starting
    with '#' are ignored.
    
    An example file would be:
        
        [plan1]
        beh1.x = 10
        beh2.y = 20
        
        [plan2]
        beh1.x = 20
    
    This file initialises two agents, one with plan1 and the other with plan2.
    Additionally, the attribute 'x' of behaviour 'beh1' of the first agent is
    set to 10, and attribute 'y' of behaviour 'beh2' to 20. For the second
    agent, the attribute 'x' of behaviour 'beh1' is set to 20.
"""

# Jython compatibility
from __future__ import nested_scopes
from POSH.jython_compat import *

import sys
import getopt
import time
import random

from POSH.utils import is_library, is_plan, get_library_file, \
    default_agent_init, default_world_script, run_world_script
from POSH.agentinitparser import parse_agent_init_file
from POSH import create_agents
from POSH.logbase import setup_console_logging, INFO, DEBUG
from POSH.behaviour import Behaviour

# private constants
_behaviour_classes = (Behaviour, )

class Agent():
   def __init__(self):
       self.random = random

def _is_behaviour_class(c):
    """Returns if the given class object is a subclass of a behaviour class.
    
    @param c: A class object
    @type c: class
    @return: If the given class is a subclass of L{POSH.Behaviour}
    @rtype: bool
    """
    for behaviour_class in _behaviour_classes:
        try:
            if issubclass(c, behaviour_class) and c != behaviour_class:
                return True
        except TypeError:
            # will be raise whenever c is not an object of type classobj
            pass
    return False

def mainx(argv = None):
    library = modname = 'olrun'
    behaviour_mod = None
    behaviour_classes = []
    try:
            behaviour_mod = __import__('%s.%s.%s' % \
                                       ('library', library, modname))
            #print(behaviour_mod)
    except Exception:
            # ignore any non-importable python file
            pass
    behaviour_mod = getattr(getattr(behaviour_mod, library), modname)
    #print(behaviour_mod, dir(behaviour_mod))
    # add all behaviour class objects in module to list
    mod_objs = [getattr(behaviour_mod, objname) \
                    for objname in dir(behaviour_mod)]
    found_classes = filter(_is_behaviour_class, mod_objs)
    

    #print(found_classes)
    class_names = ', '.join([c.__name__ for c in found_classes])
    #print(class_names)
    behaviour_classes += found_classes

    # remove double entries
    behaviour_set = {}
    for behaviour_class in behaviour_classes:
        behaviour_set[behaviour_class] = None
    return behaviour_set.keys()


def main(argv = None):
    a = Agent()
    bm = __import__('library.olrun.olrun')
    mo = bm.olrun.olrun.OlrunBehaviour
    b = mo(a)

    #beh = mainx()
    #print(beh)
    #b = beh[0](None)
    #b = OlrunBehaviour(None)
    b.prep4Task(1.0)
    b.prep4Next()
    return 0


if __name__ == '__main__':
    if (is_jython):
        print("jyposh!")
        import jyposh
#    jyposh.compile_java()
    exit_code = main()
    # only call sys.exit() if there was some error. Otherwise, sys.exit() would
    # cause all background threads to be killed. This leads to problems if
    # the world initialisation script spawns a separate thread and returns,
    # while initialising and running the agents in the background (e.g. MASON).
    # Calling sys.exit(0) in such a case would kill the Java GUI thread and
    # with it MASON.
    if exit_code != 0:
        sys.exit(exit_code)
