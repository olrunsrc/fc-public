from __future__ import nested_scopes

from POSH.logbase import LogBase


DEBUG_SCHEDULE_TIME = 5 # For now in seconds
DEFAULT_VIEWTIME = 0.5 # How long item lasts on BB in seconds

# Globals
global_symbol_counter = 0 # Depreciated by using a generator for gensym

# Helper Functions

# Gensym generates a unique symbol name
def gensym(prefix = "G"):
    global global_symbol_counter
    symbol = str(prefix) + str(global_symbol_counter)
    if global_symbol_counter >= 999999999:
        global_symbol_counter = 0
    else:
        global_symbol_counter += 1
    return symbol

class Base(LogBase):    
    def __init__(self, agent, **kw):
        # do not declare any log name as we don't know it
        LogBase.__init__(self, agent, '')
        self.agent = agent  


class Element(Base):
    
    def __init__(self,
                 trigger = None,
                 drive_name = "non_assigned_element",
                 **kw):
        Base.__init__(self, **kw) # Call the ancestor init
        self.trigger = trigger
        self.drive_name = drive_name
