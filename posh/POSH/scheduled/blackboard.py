from __future__ import nested_scopes                    

from basic import *
from messages import *
from generic import *
        
from POSH.utils import current_time

class Blackboard(Base):
    def __init__(self, **kw):
        # Call ancestor init with the remaining keywords
        # Base.__init__(self, **kw)
        # Initialize blackboard as a list for now
        self.__bb = []
        self.__bb_rp = [] # For "request" and "pending"
    # Make a copy of the bb for external use
    def copy_bb(self):
        tempbb = self.__bb + self.__bb_rp
        tempbb.sort()
        return tempbb
    def add_item(self, new_bbitem):
        # Add the items with built in Schwartzian transform :)
        if new_bbitem.command in ("request", "pending"):
            self.__bb_rp.insert(0, (new_bbitem.timeout, new_bbitem))
        else:
            self.__bb.insert(0, (new_bbitem.timeout, new_bbitem))
    # Finds a list of item that satisfies the criteria
    # ||TODO|| make this more general.
    def find_items(self,
                   tag,
                   result = None):
        #found_items = []
        # This sucks. Can we functionalize it with filter()?
        #for item in self.__bb:
        #    if item.tag == tag:
        #        if result != None:
        #            if item.result == result:
        #                found_items.append(item)
        #return found_items

        def find_filter(item):
            bbobj = item[1]
            if bbobj.tag == tag:
                if result:
                    if item.result == result:
                        return 1
                    else:
                        return 0
                else:
                    return 1
            else:
                return 0

        return filter(find_filter, self.__bb + self.__bb_rp)
            
        
    # Returns the last item with the supplied tag and
    # command
    def find_prereq(self,
                    command,
                    tag):
        #print "Searching BB for " + str(command) + " and " + str(tag)
        if command in ("request", "pending"):
            list = self.__bb_rp
        else:
            list = self.__bb

        for (timeout, item) in list:
            if item.tag == tag and item.command == command:
                #print "Found!"
                return item
        #print "None Found"
        return None    
    
    
    
#; check-bb trims bb (send-sol adds to it) and also sends things to the
#; action scheduler via the schedule.  "viewtime" is how long a message
#; from this stays on the bb, default is 10 minutes.  Note that this is
#; trimmed in real-time, regardless of whether the agent is running in
#; real time.
    def check_bb(self, timeout_interval = DEFAULT_VIEWTIME):
        now = current_time() # Get and store the time now
        temp_stack = [] # The temp stack to put items
	
        
        self.__bb.sort() # Sorting the bb pretty much reverses it
        self.__bb_rp.sort()

        # This cleans expired items the normal bb
        deleted = 0
        for i in range(len(self.__bb)):
            try:
                itemtimeout = self.__bb[i][0]
                if itemtimeout > 0 and itemtimeout < now:
		    #pass
                    del(self.__bb[i + deleted])
                    deleted += 1
                elif itemtimeout >= now:
                    break
            except:
                pass #Should raise error

        deleted = 0
        # This cleans expired items the request/pending bb
        for i in range(len(self.__bb_rp)):
            try:
                (itemtimeout, item) = self.__bb_rp[i]
                if itemtimeout > 0 and itemtimeout < now:
                    self.add_item(Bbitem(command = "cancel",
                                         action = item.action,
                                         drive_name = item.drive_name,
                                         value = item.value,
                                         timeout = now + timeout_interval,
                                         agent = item.agent))
                    del(self.__bb_rp[i + deleted])
                    deleted += 1
                elif itemtimeout >= now:
                    break
            except:
                pass # Should raise error

        # Now process each request/pending item
        for (itemtimeout, item) in self.__bb_rp:
            item.schedule_item(timestamp = now)

        # And reset the request/pending board
        self.__bb_rp = []

        # Now everything is done. Reverse the bb to make searching
        # prereqs faster
        self.__bb.reverse()

        # The commented out code below has been redone for performance
        ###
        # This filter checks each item it gets for timeout or
        # for requests and pending bbitems. Returning 1 means
        # that the item can stay on the BB, while 0 means that
        # it will not be returned.
        #def proc_bbitems(this_item):
        #    if this_item.timeout > 0 and this_item.timeout < now:
        #        if this_item.command == "request":
        #            temp_stack.insert(0,Bbitem(command = "cancel",
        #                                       action = this_item.action,
        #                                       drive_name = \
        #                                         this_item.drive_name,
        #                                       value = this_item.value,
        #                                       timeout = now + \
        #                                         timeout_interval,
        #                                       agent = this_item.agent))
        #        return 0
        #    elif this_item.command in ("request", "pending"):
        #        this_item.schedule_item(timestamp = now)
        #        return 0
        #    else:
        #        return 1
            
            
        # Run the bb through the filter and pass the revised bb
        # onto the tempstack
        #temp_stack.extend(filter(proc_bbitems, self.__bb))
        # Assign the temp stack as the new BB
        #self.__bb = temp_stack
        
