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

from POSH.utils import is_library, is_plan, get_library_file, \
    default_agent_init, default_world_script, run_world_script
from POSH.agentinitparser import parse_agent_init_file
from POSH import create_agents
from POSH.logbase import setup_console_logging, INFO, DEBUG


# there is probably a better way to do this, but we need a few libraries which require compiling
#from POSH.utils import compile_mason_java
   

class Usage(Exception):
    def __init__(self, msg):
        self.msg = msg
    def __str__(self):
        return repr(self.msg)


def process_options(argv):
    """Parses the command line options and returns them.
    
    The are returned in the order help, verbose, world_file, world_args,
    agent_file, plan_file. help and verbose are boolean variables. All the
    other variables are strings. If they are not given, then an empty string
    is returned.
    """
    # default values
    help, verbose = False, False
    world_file, world_args, agent_file, plan_file, library = '', '', '', '', ''
    # parse options
    try:
        opts, args = getopt.getopt(sys.argv[1:], 
                                   "hvw:a:i:p:",
                                   ["help", "verbose", 
                                    "init-world-file=", "init-world-args=",
                                    "init-agent-file=", "plan-file="])
    except getopt.error, msg:
        raise Usage(msg)
    # process options
    for o, a in opts:
        if o in ('-h', '--help'):
            help = True
        elif o in ('-v', '--verbose'):
            verbose = True
        elif o in ('-w', '--init-world-file'):
            world_file = a
        elif o in ('-a', '--init-world-args'):
            world_args = a
        elif o in ('-i', '--init-agent-file'):
            agent_file = a
        elif o in ('-p', '--plan-file'):
            plan_file = a
        else:
            raise Usage("unrecognised option: %s" % o)
    if help:
        return help, False, '', '', '', '', ''
    # get library from only arguments
    if len(args) != 1:
        raise Usage("requires one and only one argument (the library); plus optional options")
    library = args[0]
    if not is_library(library):
        raise Usage("cannot find specified library '%s'" % library)
    # check for option consistency
    if agent_file != '' and plan_file != '':
        raise Usage("agent initialisation file and plan file cannot be" \
                    "specified simultaneously")
    if plan_file != '' and not is_plan(library, plan_file):
        raise Usage("cannot find specified plan '%s' in library '%s'" % \
                    (plan_file, library))
    if agent_file == '' and plan_file == '' and \
       default_agent_init(library) == '':
        raise Usage("no default agent initialisation file for " \
            "library '%s', please specify one" % library)
    # all fine
    return help, verbose, world_file, world_args, agent_file, plan_file, library


def init_world(world_file, library, world_args, agents_init, verbose):
    """Calls utils.run_world_script() to initialise the world and returns the
    worls object.
    """
    if verbose:
        print "- initialising world"
    if world_args == '':
        world_args = None
    # find which world script to run
    if world_file != '':
        if verbose:
            print "running '%s'" % world_file
        return run_world_script(world_file, library, world_args, agents_init)
    if verbose:
        print "no default world initialisation script"
    return None, False


def main(argv = None):
    
    # There must be a beter way to do this... see jyposh.py and utils
#    compile_mason_java() 

    # process command line options
    if argv == None:
        argv = sys.argv
    try:
        help, verbose, world_file, world_args, agent_file, plan_file, library = \
            process_options(argv)
    except Usage, err:
        print >>sys.stderr, err.msg
        print >>sys.stderr, "for help use --help"
        return 2
    if help:
        print __doc__
        return 0

    # activate logging. we do this before initialising the world, as it might
    # use this logging facility
    if verbose:
        setup_console_logging(DEBUG)
    else:
        setup_console_logging(INFO)

    # read agent initialisation. this needs to be done before initialising
    # the world, as the agent initialisation needs to be give to the world
    # initialisation script
    if verbose:
        print "- collect agent initialisation options"
    if plan_file == '':
        if agent_file == '':
            agent_file = default_agent_init(library)
        if verbose:
            print "reading initialisation file '%s'" % agent_file
        try:
            agents_init = parse_agent_init_file(agent_file)
        except Exception, e:
            try:    #try again with path
                agents_init = parse_agent_init_file(get_library_file(library, agent_file))
            except Exception, e:  
                print "reading agent initialisation file failed"
                print "----"
                raise
    else:
        if verbose:
            print "create single agent with plan '%s'" % plan_file
        agents_init = [(plan_file, {})]
    if verbose:
        print "will create %d agent(s)" % len(agents_init)
    
    # initialise the world
    if world_file == '':
        world_file = default_world_script(library)
    try:
        world, created_agents = init_world(world_file, library, 
                                           world_args, agents_init, verbose)
    except Exception, e:
        try:     #try again with path
            world, created_agents = init_world(get_library_file(library, world_file), library, 
                                               world_args, agents_init, verbose)
        except:
            print "world initialisation failed"
            print "----"
            raise
    if created_agents:
        if verbose:
            print "- world initialisation script indicated that it created "\
                  "agents. nothing more to do."
        return 0
    
    # create the agents
    if verbose:
        print "- creating agent(s)"
    try:
        agents = create_agents(library,
                               agents_init = agents_init,
                               world = world)
    except Exception, e:
        print "launch.py: creating agent(s) failed, see following error"
        print "----"
        raise
    
    # start the agents
    if verbose:
        print "- starting the agent(s)"
    for agent in agents:
        agent.startLoop()
    # check all 0.1 seconds if the loops are still running, and exit otherwise
    loops_running = 1
    while loops_running > 0:
        time.sleep(0.1)
        loops_running = 0
        for agent in agents:
            if agent.loopStatus()[0]:
                loops_running += 1
    
    if verbose:
        print "- all agents stopped"

if __name__ == '__main__':
    if (is_jython):
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