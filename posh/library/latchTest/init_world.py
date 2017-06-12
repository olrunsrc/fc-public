"""TODO: PR -- Comment.
"""

# make sure that it is called from jython
from POSH.jython_compat import *
if not is_jython:
    raise Exception, "Needs to be called from Jython, not Python"

from library.mason.mason import MASONWorld
from java.awt import Color

# the whole purpose of this library is to run stats, so we need to set the AgentBase class (from POSH/agent_base.py) to handle that
import POSH.profiler
POSH.profiler.Profiler.turnOnProfiling()

# the following constants are used when initialising mason by creating an
# instance of MASONWorld 

# the name of the simulation
SIMNAME = 'latchTest:  Eat and Drink'

# the size of the field
FIELD_WIDTH = 600
FIELD_HEIGHT = 600

# the fields that agents reside in
FIELDS = ('agents', 'food', 'water')

# maximum display size
DISPLAY_SIZE = 750

# background colour
BACKGROUND_COLOUR = Color(0, 250, 0)

# access the world object provided by the caller of this script
library = world.library()
agents_init = world.agentsInit()
# signal that this script creates and runs the agents (by calling MASONWorld)
world.createsAgents()
# initialises MASON, the GUI, and runs it.
mason = MASONWorld(library, agents_init, SIMNAME, FIELD_WIDTH, FIELD_HEIGHT,
                   FIELDS, DISPLAY_SIZE, BACKGROUND_COLOUR)
mason.run()


