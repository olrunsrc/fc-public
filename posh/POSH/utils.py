"""Module with helper function to find libraries, load behaviours, ..."""
from __future__ import nested_scopes
from jython_compat import *

import os
import sys
import time

# get main configuration
import config

# to identify behaviours
from behaviour import Behaviour

# private constants
_behaviour_classes = (Behaviour, )

# counters
_agent_id = 0

class World:
    """The World class that is used to communicate with the world
    initialisation script.
    
    Upon running the world initialisation script, using the L{run_world_script},
    an instance of this class, named 'world' is given to the script. The
    script can use this instance to gather information on how the world is to
    be initialised, and can return the world object and other information to
    the instance that calls this script.
    """
    def __init__(self, library, world_args = None, agents_init = None):
        """Initialises an instance with the neccessary information.
        
        @param library: name of the behaviour library that is to be used.
        @type library: string
        @param world_args: arguments to be given to the world initialisation
            script.
        @type world_args: string
        @param agents_init: structure containing information to initialise
            the agents.
        @type agents_init: as returned by
            L{POSH.agentinitparser.parse_agent_init_file}
        """
        self._library = library
        self._args = world_args
        if agents_init == None:
            self._agents_init = []
        else:
            self._agents_init = agents_init
        self._creates_agents = False
        self._world = None
    
    def args(self):
        """Returns the arguments for customised world initialisation.
        
        If no arguments are given, None is returned.
        
        @return: arguments for world initialisation.
        @rtype: string or C{None}
        """
        return self._args
    
    def set(self, x):
        """Sets the world object for use when initialising the agents.
        
        The world object given to this method is given to the agents upon
        initialisation.
        
        @param x: The world object
        @type x: any
        """
        self._world = x
    
    def library(self):
        """Returns the behaviour library name that the agents are to use.
        
        @return: name of the library to use
        @rtype: string
        """
        return self._library
    
    def agentsInit(self):
        """Returns the agents initialisation structure.
        
        @return: agents initialisation structure.
        @rtype: as returned by L{POSH.agentinitparser.parse_agent_init_file}
        """
        return self._agents_init
    
    def createsAgents(self):
        """Specifies that the agents are created an run by the world
        initialisation script.
        
        By default, the world initialisation script is only responsible for
        setting up the world, and eventually returning the world object to
        initialise the agents with. Calling this method from the world
        initialisation script indicates that both creation and running the
        agents is performed by the world initialisation script. For this
        purpose, the script can use L{library()} and L{agentsInit()}.
        """
        self._creates_agents = True

# to provide current_time()
if sys.platform == "win32":
    # On Windows, the best timer is time.clock()
    current_time = time.clock
else:
    # On most other platforms the best timer is time.time()
    current_time = time.time

def get_root_path():
    """Returns the root path that POSH is installed in. Assumes that this
    module resides in the root path.
    
    @return: Root path
    @rtype: string
    """
    return os.path.split(config.__file__)[0]

def _get_library_path(library = None):
    """Returns the path to the behaviour library. If no library is given,
    the the base path of the library is returned.
    
    @param library: The library to return the path for
    @type library: string
    @return: The base library path or the path to the given library
    @rtype: string
    """
    if library:
        return os.path.join(get_root_path(), config.LIBRARYPATH, library)
    else:
        return os.path.join(get_root_path(), config.LIBRARYPATH)

def _get_plans_path(library):
    """Returns the path to the plans of the given behaviour library.
    
    @param library: The library to return the path for
    @type library: string
    @return: The path to the plans
    @rtype: string
    """
    return os.path.join(_get_library_path(library), config.PLANSPATH)

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

def get_plans(library):
    """Returns a list of available plans for the given behaviour library.
    
    The plans need to be located at library/PLANS and end in PLANENDING.
    
    @param library: The library to return the list of plans for
    @type library: string
    @return: A list of plans without file ending
    @rtype: sequence of strings
    """
    planpath = _get_plans_path(library)
    try:
        dirlist = os.listdir(planpath)
    except OSError:
        return []
    planfilter = lambda f: \
        (not f[:1] == '.') and \
        f[-len(config.PLANENDING):].lower() == config.PLANENDING.lower() and \
        os.path.isfile(os.path.join(planpath, f))
    planfiles = filter(planfilter, dirlist)
    # remove the endings of the files
    return [filename[:-len(config.PLANENDING)] for filename in planfiles]

def get_plan_file(library, plan):
    """Returns the plan file name for the given library and plan
    
    @param library: The library that the plan is from
    @type library: string
    @param plan: The name of the plan (without the .lap ending)
    @type plan: string
    @return: The filename with full path of the plan
    @rtype: string
    """
    return os.path.join(_get_plans_path(library), 
                        "%s%s" % (plan, config.PLANENDING))
    
def get_library_file(library, file):
    """Returns the file name for the given library (not for plans, see above)
    
    @param library: The library that the file is in
    @type library: string
    @param plan: The name of the file (including any needed ending)
    @type plan: string
    @return: The filename with full path 
    @rtype: string
    """
    return os.path.join(_get_library_path(library), 
                        "%s" % (file))   

def is_plan(library, plan):
    """Returns if the given plan of the given library exists.
    
    This method only checks if the plan file exists, not if its syntax is
    correct.
    
    @param library: The library that the plan is from
    @type library: string
    @param plan: The name of the plan (without the .lap ending)
    @type plan: string
    @return: If the plan exists (i.e. is a file)
    @rtype: bool
    """
    return os.path.isfile(get_plan_file(library, plan))

def get_libraries():
    """Returns a list of available behaviour libraries.

    The function assumes that any directory in the libraries path that
    does not start with an '.' is a behaviour library.
    
    @return: List of libraries
    @rtype: sequence of strings
    """
    library_path = _get_library_path()
    dirlist = os.listdir(library_path)
    libraryfilter = lambda d: \
        (not d[:1] == '.') and \
        os.path.isdir(os.path.join(library_path, d))
    return filter(libraryfilter, dirlist)

def is_library(library):
    """Returns if the given library is a valid behaviour library.
    
    This is the case if the following requirements are met:
        - there is a directory with the given name in LIBRARYPATH
        - the directory can be loaded as a module, which requires it to at least
          contain a __init__.py file
    
    @param library: Name of the library to check
    @type library: string
    @return: If the library is a valid library
    @rtype: bool
    """
    try:
        lib_module = __import__("%s.%s" % (config.LIBRARYPATH, library))
    except ImportError:
        return False
    return True

def get_behaviours(library, log = None):
    """Returns a sequence of classes, containing all behaviour classes that
    are available in a particular library.
    
    The method searches the behvaiour subclasses by attempting to import
    all file in the library ending in .py, except for the WORLDSCRIPT, and
    search through all classes they contain to see if they are derived from
    any behaviour class.
    
    If a log object is given, then logging output at the debug level is
    produced.
    
    @param library: Name of the library to find the classes for
    @type library: string
    @param log: A log object
    @type log: as from the Python logging module
    @return: The list of classes
    """
    # get list of python files is behaviour library
    if log != None:
        log.debug("Scanning library '%s' for behaviour classes" % library)
    librarypath = _get_library_path(library)
    try:
        dirlist = os.listdir(librarypath)
    except OSError:
        return []
    pyfilter = lambda f: \
        (not f[:1] == '.') and \
        f[-3:].lower() == '.py' and \
        os.path.isfile(os.path.join(librarypath, f))
    pylist = filter(pyfilter, dirlist)
    # try to load the library modules to collect the behaviour classes
    behaviour_classes = []
    for pyfile in pylist:
        if pyfile == config.WORLDSCRIPT:
            continue
        modname = pyfile[:-3]
        # try to load the module
        try:
            behaviour_mod = __import__('%s.%s.%s' % \
                                       (config.LIBRARYPATH, library, modname))
        except Exception:
            # ignore any non-importable python file
            if log != None:
                log.debug("Loading the file '%s' failed" % pyfile)
            continue
        # workaround to get the real module rather than its top-level parent
        # (see __import__ documentation)
        behaviour_mod = getattr(getattr(behaviour_mod, library), modname)
        # add all behaviour class objects in module to list
        mod_objs = [getattr(behaviour_mod, objname) \
                    for objname in dir(behaviour_mod)]
        found_classes = filter(_is_behaviour_class, mod_objs)
        if log != None:
            class_names = ', '.join([c.__name__ for c in found_classes])
            log.debug("Found %d behaviour class(es) in '%s': %s" % \
                      (len(found_classes), pyfile, class_names))
        behaviour_classes += found_classes
    # remove double entries
    behaviour_set = {}
    for behaviour_class in behaviour_classes:
        behaviour_set[behaviour_class] = None
    return behaviour_set.keys()

def default_agent_init(library):
    """Returns the default agent initialisation file filename for the given
    library.
    
    If no such file exists, and emtpy string is returned.
    
    @param library: Name of the library
    @type library: string
    @return: Agent initialisation file filename, or '' if not found
    @rtype: string
    """
    agent_init = os.path.join(_get_library_path(library), config.AGENTINIT)
    if os.path.isfile(agent_init):
        return agent_init
    else:
        return ''

def unique_agent_id():
    """Returns a unique agent id string of the form 'Axx', where xx is
    an increasing number, starting from 00.
    
    If more than 99 agents are created, the string length adjusts to the
    length of the number.
    
    @return: Unique agent id
    @rtype: string
    """
    global _agent_id
    _agent_id += 1
    return 'A%02d' % (_agent_id - 1)

def default_world_script(library):
    """Returns the default world initialisation script for the given library.
    
    If no script was found, then an empty string is returned.
    
    @param library: Name of the library
    @type library: string
    @return: World initialisatoin script filename, of '' if not found
    @rtype: string
    """
    world_script = os.path.join(_get_library_path(library), config.WORLDSCRIPT)
    if os.path.isfile(world_script):
        return world_script
    else:
        return ''

def run_world_script(script_file, library,
                     world_args = None, agents_init = None):
    """Runs the given file to initialise the world and returns the world object
    and if the world initialisation script creates and runs the agents
    itself.
    
    This method creates an instance of the class L{World} and makes it
    accessible to the world initialisation script under the instance name
    'world'. Using this instance, the world intialisation script can
    cimmunicate with this function.
    
    @param script_file: The filename of the script to run
    @type script_file: string
    @param library: name of the behaviour library to use by agents
    @type library: string
    @param world_args: arguments given to the world initialisation script
    @type world_args: string
    @param agents_init: agent initialisation information structure
    @type agents_init: as returned by
        L{POSH.agentinitparser.parse_agent_init_file}
    @return: tuple (world object, if script created and ran agents)
    @rtype: (any, bool)
    @raise IOError: If it cannot file the script
    @raise Exception: If the script causes an exception
    """
    w = World(library, world_args, agents_init)
    # world instance is given as local variable
    # note that if the global and local variables (when calling execfile)
    # refer to a different object, then some strange things happen as soon as
    # the script starts importing other modules (don't know why). So both
    # arguments have to refer to the same object!
    variables = {'world' : w}
    execfile(script_file, variables, variables)
    return (w._world, w._creates_agents)

#There must be a better way (& place) to do this...
#Note, because jython (or maybe mason.jar) is currently 1.4, we need to be sure to compile our classes in 1.4 too
def compile_mason_java():
      import os
      ext1='.java'
      ext2='.class'
      dir = os.path.join(get_root_path(), config.MASONPATH)
      dir=get_root_path()+'/platform_files/MASON/'
      java_src=filter((lambda str:str!="__init__"),map((lambda str:str[:len(str)-len(ext1)]),filter((lambda str: str.endswith(ext1)),os.listdir(dir))))
      classes=filter((lambda str:str!="__init__$py"),map((lambda str:str[:len(str)-len(ext2)]),filter((lambda str:str.endswith(ext2)),os.listdir(dir))))
   
      if java_src!=classes:
            cmd = 'javac -target 1.4 -source 1.4 -cp %smason.jar %s*.java' % (dir,dir)
            os.system(cmd)