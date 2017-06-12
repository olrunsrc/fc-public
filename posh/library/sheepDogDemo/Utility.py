from __future__ import nested_scopes
# behaviour base for POSH or SPOSH behaviours
from POSH import Behaviour
from config import *

class Utility(Behaviour):
    
    def __init__(self, agent):
        Behaviour.__init__(self,agent,(),('s_fail', 's_succeed'))

    def s_fail(self):
        return False
    
    def s_succeed(self):
        return True
