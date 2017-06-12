from __future__ import nested_scopes

from basic import Element

class Drive_Element(Element):
    
    def __init__(self,
                 drive_root = "fixme_de",
                 drive_root_rep = "fixme_dep",
                 frequency = -1,
                 frequency_rep = None,
                 last_fired = 0,
                 drive_collection = "not_my_drive_collection",
                 **kw):
        # Call ancestor init with the remaining keywords
        Element.__init__(self, **kw)
        self.drive_root = drive_root
        self.drive_root_rep = drive_root_rep
        self.frequency = frequency
        self.frequency_rep = frequency_rep
        self.last_fired = last_fired
        self.drive_collection = drive_collection
    def ready(self):
        if self.trigger:
            return self.trigger.trigger()
        else:
            return Ture
