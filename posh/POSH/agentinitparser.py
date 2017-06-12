"""Module to parse agent initialisation file.

The initialisation file has the following format::
    
    [plan1]
    beh1.attr1 = value1
    beh2.attr1 = value2
    
    [plan2]
    beh1.attr1 = value3
    beh2.attr2 = value4
    
Such a file specifies two agents to be creates, one using 'plan1', and the
second using 'plan2'. The first agent is initialised as follows: attribute
attr1 of behaviour beh1 is set to value1, and attribute attr1 of behaviour
beh2 is set to value2. The second agent is initialised in a similar way.

In more detail, the agent initialisation is specified in a line-wise fashion.
Comments (starting with a '#') and empty lines are ignored. Every agent block
in the file start with its plan, given by '[plan]', where 'plan' can be any
string _without_ spaces. After that an aribtrary number of attributes (one per
line) can be given for each behaviour of the agent. An attribute is specified
by 'behaviour.attrribute = value', where 'behaviour' and 'attribute' are
names of the behaviour and the attribute, respectively, and 'value' is the
value that the attribute is set to. The following regular expressions
determine valid names and values::
    
    behaviour and attribute names  [a-zA-Z_][a-zA-Z0-9_]*
    values:
        integer                    \-?[0-9]+
        float                      \-?(\d*\.\d+|\d+\.)([eE][\+\-]?\d+)?
        boolean                    ([Tt]rue|[Ff]alse)

The values are automatically converted into the recognised type. If no type is
recognised, then they are assigned to the attributes as strings.
"""

# Jython compatibility
from __future__ import nested_scopes
from jython_compat import *

import re

class AgentInitParseError(Exception):
    def __init__(self, msg):
        self.msg = msg
    def __str__(self):
        return repr(self.msg)

# symbols
_int_matcher = re.compile(r"\-?[0-9]+$")
_float_matcher = re.compile(r"\-?(\d*\.\d+|\d+\.)([eE][\+\-]?\d+)?$")
_bool_matcher = re.compile(r"([Tt]rue|[Ff]alse)$")
_identifier_matcher = re.compile(r"[a-zA-Z_][a-zA-Z0-9_]*$")

# structures (whole lines, except for comments)
_plan_matcher = re.compile(r"\[(\S+)]$")
_attribute_matcher = re.compile(r"(\S+)\.(\S+)\s*=\s*(.+)$")

def str_to_value(s):
    """Converts the given string to the most likely type that it represents.
    
    It tests the types in the following order: int, float, bool. If none of 
    these match, then the string is returned as a string.
    
    @param s: String to convert
    @type s: string
    @return: Converted value
    @rtype: float, int, bool, or string
    """
    s = s.strip()
    if _int_matcher.match(s):
        return int(s)
    elif _float_matcher.match(s):
        return float(s)
    elif _bool_matcher.match(s):
        return (s.lower() == 'true')
    return s

def _strip(s, chars):
    """Removes leading and training characters from that given string that
    are within the string 'chars'.
    
    This functionality is already supported by str.strip(.), but only after
    version 2.2.2, and thus is not supported in the Jython implementation that
    I'm currently working with.
    """
    # leading characters
    while len(s) > 0 and s[0] in chars:
        s = s[1:]
    # trailing characters
    while len(s) > 0 and s[-1] in chars:
        s = s[:-1]
    return s


def parse_agent_init_file(agent_init_file):
    """Returns a data structure containing the content of the file.
    
    See the module docstring for the accepted file format. The returned data
    structure is a sequence with one entry per agent, with is a pair of a
    string for the plan and a dictionary for the
    (behaviour, attribute) -> value assignment.
    
    @param agent_init_file: Filename of the agent initialisation file
    @type agent_init_file: string
    @return: Data structure representing content of the file
    @rtype: dictionary (behaviour, attribute) -> value
    @raise IOError: when the file cannot be read
    @raise AgentInitParseError: when the syntax of the file is incorrect
    """
    try:
        f = open(agent_init_file, 'r')
    except:
        print "Cannot access agent init file (tried %s)." % agent_init_file 
        sys.stdout.flush()
        raise

    plan = ''
    current_attributes = {}
    line_nr = 0
    agents_init = []
    for line in f.readlines():
        line_nr += 1
        # clean comments, newlines, leading and trailing spaces
        comment_pos = line.find('#')
        if comment_pos != -1:
            line = line[:comment_pos]
        line = _strip(line, ' \t\n\r\f\v')
        # ignore empty lines
        if line == '':
            continue
        # new agent starts with plan
        if _plan_matcher.match(line):
            if plan != '':
                agents_init.append((plan, current_attributes))
            plan, current_attributes = line[1:-1], {}
        else:
            # not a plan, needs to be attribute assignment
            matched_attr = _attribute_matcher.match(line)
            if matched_attr == None:
                raise AgentInitParseError("line %d: unrecognised syntax '%s'" % \
                                          (line_nr, line))
            elif plan == '':
                # attributes before plan
                raise AgentInitParseError("line %d: [plan] expected" % line_nr)
            behaviour, attribute, value = matched_attr.groups()
            # check if behaviour and attribute are identifiers
            if _identifier_matcher.match(behaviour) == None or \
               _identifier_matcher.match(attribute) == None:
                raise AgentInitParseError("line %d: '%s.%s' has incorrect syntax" % \
                                          (line_nr, behaviour, attribute))
            # check what the value could be
            current_attributes[(behaviour, attribute)] = str_to_value(value)
    
    if plan != '':
        agents_init.append((plan, current_attributes))
    if len(agents_init) == 0:
        raise AgentInitParseError("no agents specified in initialisation file")
    return agents_init