from __future__ import nested_scopes

from basic import *

class Message(Base):
    def __init__(self,
                 command = None,
                 tag = None,
                 action = None,
                 value = None,
                 timestamp = None,
                 **kw):
        # Call ancestor init with the remaining keywords
        Base.__init__(self, **kw)
        self.command = command
        self.tag = tag
        self.action = action
        self.value = value
        self.timestamp = timestamp
