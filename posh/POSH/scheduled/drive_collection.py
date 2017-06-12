
from __future__ import nested_scopes

from complex import *

class Drive_Collection(Complex):
    def __init__(self, goal = None, realtime = 1 , **kw):
        # Call ancestor init with the remaining keywords
        Complex.__init__(self, **kw)
        self.goal = goal
        self.realtime = realtime
