
from __future__ import nested_scopes

from basic import *
from generic import *
from complex import *
from action_pattern import *
from blackboard import *
from competence import *
from competence_element import *

from drive_collection import *
from drive_element import *

from messages import *
from schedule import *
from sense import *


class Schedule(Base):
    def __init__(self, **kw):
        # Call ancestor init with the remaining keywords
        # Base.__init__(self, **kw)
        # Initialize schedule as a list
        self.__schedule = []
        
    # Add the item to the schedule
    def add_item(self, new_item):
        self.__schedule.append(new_item)

    # Make a copy of the schedule
    def copy_schedule(self):
        return self.__schedule[:]

    # The action scheduler
    def run(self, timeout_interval = DEFAULT_VIEWTIME):
        now = current_time()
        def proc_items(this_item):
            if this_item.timeout and (this_item.timeout < now):
                this_item.send(content = Content(command = "cancel",
                                                 tag = this_item.tag,
                                                 value = this_item.value,
                                                 timestamp = now,
                                                 agent = this_item.agent),
                               timeout = timeout,
                               timeout_interval = timeout_interval)
                return 0
            elif this_item.ready():
                if this_item.fire(timeout = now,
                                  timeout_interval = timeout_interval) \
                                  != "preserve":
                    return 0
                else:
                    return 1
            else:
                return 1
        self.__schedule = filter(proc_items, self.__schedule)

    # The action schedule for drives
    def run_drive(self,
                  drive_name,
                  timeout_interval = DEFAULT_VIEWTIME):
        now = current_time()
	
       
    
        # We need to make it a list to work around this closures issue
        drive_item_found = [0]
        def proc_items(this_item):

            if this_item.timeout and (this_item.timeout < now):
                this_item.send(content = Content(command = "cancel",
                                                 tag = this_item.tag,
                                                 action = this_item.action,
                                                 value = this_item.value,
                                                 timestamp = now,
                                                 agent = this_item.agent),
                               timestamp = now,
                               timeout_interval = timeout_interval)
                return 0
            elif (this_item.drive_name == drive_name) and this_item.ready():
                drive_item_found[0] = 1
                # If the item is flagged with "preserve" then don't
                # remove it from the schedule
                if this_item.fire(timestamp = now,
                                  timeout_interval = timeout_interval) == \
                                  "preserve":
                    return 1
                else:
                    return 0
            else:
                # ||TODO|| Add some debug stuff on when actions are not
                # firing because they are not ready
                return 1
        self.__schedule = filter(proc_items, self.__schedule)

    
        return drive_item_found[0]
        
