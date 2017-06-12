from __future__ import nested_scopes

from basic import *
#from generic import *
from types import *
from message import *
from blackboard import *

class Prereq(Message):
    def __init__(self, **kw):
        # Call ancestor init with the remaining keywords
        Message.__init__(self, **kw)
        # Undefine unused varibles
        del self.action
        del self.value
        del self.timestamp




