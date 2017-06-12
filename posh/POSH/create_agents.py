"""Functions to create new agents.

The functions were taken out of the utils module to avoid cyclic imports.
"""

# utility functions
from utils import get_plan_file

# agent bases
from strict import Agent as StrictAgent
from scheduled import Agent as ScheduledAgent

# private constants
_plan_types = ('DC', 'SDC', 'RDC', 'SRDC')
_agent_types = {'DC' : ScheduledAgent,
                'RDC' : ScheduledAgent,
                'SDC' : StrictAgent, 
                'SRDC' : StrictAgent }

def get_plan_type(planfile):
    """Returns the type of the plan of the given plan file.
    
    The type is returned as a string (e.g. 'SDC', 'RDC', SRDC'). If the type was not
    found then an empty string is returned. The plan has to be given without
    its file ending and has to be in the PLANPATH directory in the
    corresponding library.
    
    The function parses the plan line by line until it finds a plan type
    identifier after a '('. This function can fail in several ways:
        - There is a comment that has '(DC' or similar
        - The bracket is on the line before the identifier
        - Other ways that I haven't though about
    
    @param planfile: Filename of the plan file
    @type planfile: string
    @return: Type of plan, or '' if not recognised
    @rtype: string
    @raise IOError: If plan file is not found.
    """
    plan_lines = open(planfile, 'r').readlines()
    for plan_line in plan_lines:
        for plan_id in _plan_types:
            # find plan type identifier
            id_pos = plan_line.find(plan_id)
            if id_pos == -1:
                continue
            # is there a bracket before?
            bracket_pos = plan_line[:id_pos].rfind('(')
            if bracket_pos == -1:
                continue
            # only valid if there is nothing else than whitespaces between
            # the plan identifier and the bracket
            if id_pos - bracket_pos == 1 or \
               plan_line[bracket_pos + 1:id_pos].isspace():
                return plan_id
    return ''

def create_agents(library, plan = None, agents_init = None, world = None):
    """Returns a sequence of newly created agents using the given behaviour
    library.
    
    The type of agents are determined by their plan type, using the
    get_plan_type() function. If a world object is given, it is given to the
    agent upon initialisation.
    
    The function must be given either a plan file of an agents_init structure,
    but not both. If a plan is given, then a single agent is created and
    returned as a sequence of one element. agents_init is a structure as
    returned by the agentinitparser module and allows creating and
    initialisation of several agents. The agents are created one by one
    and returned as a sequence. If both a plan and agents_init are given,
    then the plan is ignored.
    
    @param library: name of the library
    @type library: string
    @param plan: name of the plan (without path and file ending)
    @type plan: string
    @param agents_init: data structure for agent initialisation
    @type agents_init: as returned by L{POSH.agentinitparser.parse_agent_init_file}
    @param world: world object, given to agents at construction
    @type world: any
    @return: the created agents
    @rtype: sequence of L{POSH.agent_base.AgentBase}
    @raise Exception: everything that can go wrong when initialising agents.
    """
    # build initialisation structure
    if agents_init == None:
        if plan == None:
            raise TypeError, "create_agent() requires either plan or " \
                             "agents_init to be specified"
        agents_init = [(plan, {})]
    # create the agents
    agents = []
    for agent_init in agents_init:
        agent_plan, agent_attributes = agent_init
        # determine agent type from plan
        plan_type = get_plan_type(get_plan_file(library, agent_plan))
        if plan_type == '':
            raise SyntaxError, \
                  "plan type of plan '%s' not recognised" % agent_plan
        agent_type = _agent_types[plan_type]
        # create agent and append to sequence
        agents.append(agent_type(library, agent_plan, agent_attributes, world))
    return agents