""" This file is for running a set of experiments using the profiler.  See documentation.html in library/latchTest
Note:  this script doesn't currently work from eclipse, and anyway runs much faster from the command line.
"""

import sys
sys.path.append('..')

from POSH.agentinitparser import parse_agent_init_file
from POSH.utils import World
from library.mason.mason import MASONWorld
from java.awt import Color
import POSH,random,os

# the whole purpose of this library is to run stats, so we need to set the AgentBase class (from POSH/agent_base.py) to handle that
import POSH.profiler
POSH.profiler.Profiler.turnOnProfiling()

library_name='latchTest'
agents_init=parse_agent_init_file(os.getcwd()+'/../library/'+library_name+'/init_agent')

trials=15
limit=5050

for i in range(trials):
    
    w=World(library_name,None,agents_init)
    
    library = w.library()
    agents_init = w.agentsInit()
    w.createsAgents()
    mason = MASONWorld(library, agents_init,'Eat and Drink',600,600,('agents','food','water'),750,Color(0,200,0))
    
    c=mason.gui.run()
    c.setWhenShouldEnd(limit)
    c.pressPlay()
    
    while c.getPlayState()==c.PS_PLAYING:
        pass
    
    POSH.utils._agent_id = 0
    
execfile(os.getcwd()+'/../POSH/profiler.py')
c.doQuit()